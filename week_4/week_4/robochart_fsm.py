'''
A prototype implementation of RoboChart state machines in Python.

A state machine is a class that inherits from StateMachine. Its variables, constants,
clocks, events and operations are declared as class annotations, and its states are
nested classes that inherit from State:

    class TurtleBot3FSMC(StateMachine):

        C: Clock                                    # clock C
        T: Const[int]                               # const T: int, given at construction
        move: Required[Operation]                   # requires IMove
        obstacle: Required[Input[Direction]]        # event obstacle: Direction

        def initial(self):                          # initial junction
            return self.Forward

        class Forward(State):

            def entry(self):                        # entry move(lvel, 0)
                yield from self.move(lvel, 0.0)

            def transitions(self):                  # checked on every cycle
                if self.C.since() >= self.T:
                    return self.Turning, self.C.reset   # target state, and action #C
                if self.obstacle.triggered():
                    def action():                   # action of the transition, defined with it
                        yield from self.move(0.0, avel)
                    return self.Avoiding, action

            def exit(self):
                ...

        class Turning(State):
            ...

Inside a state, 'self' gives access to anything declared in the machine, and to the
other states. A state with an 'initial' method is a composite state, whose substates
are its nested State classes. Entry and exit actions, the actions of transitions, and
'transitions' itself may be generators that 'yield' to take several cycles. For example,
the RoboChart action wait(3) is 'yield from self.wait(3)'.

The action of a transition is a function defined next to the transition, in 'transitions',
and returned with the target without calling it. It is called after the state has been
exited, as in RoboChart. When the action is a single call of an existing method, the method
itself can be given, as in 'self.C.reset', without calling it.

A state machine is executed by calling 'execute()' periodically, for example from the
callback of a ROS timer, which also steps its clocks.
'''

import collections.abc
import copy
import difflib
import inspect
import threading
import types
import typing
import warnings
from enum import Enum
from typing import Any, Generic, TypeVar

from pydantic import ConfigDict, TypeAdapter, ValidationError
from pydantic.errors import PydanticUserError

T = TypeVar('T')


class MachineStatus(Enum):
    # A state is being entered or exited, or the action of a transition is executing.
    # As in RoboChart, this cannot be interrupted, so transitions of enclosing states
    # are not checked till it has finished.
    BUSY = 0
    # A final state has been reached.
    FINISHED = 1


class _Final:
    def __repr__(self):
        return 'FINAL'


# Target of a transition to a final state: return FINAL from 'transitions' or 'initial'.
FINAL = _Final()


class _Exiting(BaseException):
    '''
    Thrown into the generator of a state when an enclosing state takes a transition,
    so that the state runs its exit action. It derives from BaseException so that an
    'except Exception' in user code does not catch it.
    '''


# ---------------------------------------------------------------------------------
# Declarations
# ---------------------------------------------------------------------------------

# These are type aliases, so that, for type checkers, 'K: Const[int]' declares an int.

# Declares a constant: 'K: Const[int]'. Its value is given at construction, or as a default.
type Const[V] = V

# Declares a parameter of an operation: 'lvel: Param[float]'.
type Param[V] = V

# Declares a name provided by an enclosing scope: 'cmd_vel: Required[...]'. For a state
# machine, that is the scope given as 'parent' at construction (a robotic platform or a
# controller). For an operation, that is the machine or operation calling it.
type Required[V] = V


class Clock:
    '''
    A RoboChart clock: 'C: Clock'. It is created and stepped by the state machine, once per
    timed cycle (tick). As in RoboChart, time is discrete: since(C) is the number of whole
    time units since #C, where a time unit is a whole number of ticks.
    '''

    def __init__(self):
        # Ticks since #C
        self.count = 0
        self.ticks_per_unit = None

    def step(self):
        self.count += 1

    def reset(self):
        ''' The action #C. '''
        self.count = 0

    def since(self):
        ''' The expression since(C): the number of whole time units since #C. '''
        if self.ticks_per_unit is None:
            raise RuntimeError("Clock is not used by a state machine, so it has no time unit.")
        return self.count // self.ticks_per_unit


class Input(Generic[T]):
    '''
    An input event: 'obstacle: Input[Direction]'. It is set by the environment, for example
    from a ROS callback, and consumed by the state machine. What happens to an event that is
    not consumed is given by 'keep', for example 'obstacle: Input[Direction] = Input(keep = 'cycle')':

      - 'latest' (default): it is kept till consumed, and a new value replaces it; suitable
        for sensors where only the latest reading matters;
      - 'cycle': it is cleared at the end of the cycle (call to 'execute') in which it was
        available, so it is only accepted if the machine is ready for it then;
      - 'queue': all values are kept, in order, till consumed; suitable for commands that
        must not be lost.
    '''

    KEEP = ('cycle', 'latest', 'queue')

    def __init__(self, keep = 'latest'):
        if keep not in Input.KEEP:
            raise ValueError(f"keep must be one of {', '.join(map(repr, Input.KEEP))}, but got {keep!r}.")
        self.keep = keep
        self._validate = None
        self._values = []
        # Lock of the state machine using this event, so that it is not set during a cycle.
        self._lock = None

    def __deepcopy__(self, memo):
        return Input(self.keep)

    def set(self, value = None):
        ''' Make the event available, communicating 'value'. '''
        if self._validate is not None and value is not None:
            value = self._validate(value)
        if self._lock is None:
            self._set(value)
        else:
            with self._lock:
                self._set(value)

    def _set(self, value):
        if self.keep == 'queue':
            self._values.append(value)
        else:
            self._values = [value]

    def triggered(self):
        ''' Whether the event is available. '''
        return len(self._values) > 0

    @property
    def value(self):
        ''' The value communicated, without consuming the event. '''
        return self._values[0] if self._values else None

    def take(self):
        ''' Consume the event, returning the value communicated: obstacle?obs. '''
        return self._values.pop(0) if self._values else None

    def clear(self):
        ''' Discard the event, and any values queued. '''
        self._values = []

    def _end_of_cycle(self):
        if self.keep == 'cycle':
            self.clear()


class Output(Generic[T]):
    ''' An output event: 'beep: Output[None]'. It is given, at construction, a function that implements it. '''

    def __init__(self, implementation):
        self._implementation = implementation

    def __call__(self, *value):
        ''' Communicate the event, as in beep!v. '''
        self._implementation(*value)


class _Declaration:

    def __init__(self, kind, type_, annotation):
        self.kind = kind                # var, const, param, required, clock, input, output, function, operation
        self.type = type_
        self.annotation = annotation
        self.has_default = False
        self.default = None
        self._adapter = None

    def validate(self, name, value):
        ''' Check that 'value' has the declared type of variable, constant or parameter 'name'. '''
        if self.type is Any:
            return value
        if self._adapter is None:
            try:
                self._adapter = TypeAdapter(self.type, config = ConfigDict(arbitrary_types_allowed = True, strict = True))
            except PydanticUserError:
                self._adapter = TypeAdapter(self.type)
        try:
            return self._adapter.validate_python(value)
        except ValidationError:
            raise TypeError(f"'{name}' is declared with type {_type_name(self.type)}, but was given {value!r}.") from None


def _type_name(type_):
    return getattr(type_, '__name__', repr(type_))


def _is_operation_type(annotation):
    ''' Whether the annotation is 'Operation' or 'type[Operation]'. '''
    return annotation is Operation or (typing.get_origin(annotation) is type and typing.get_args(annotation) == (Operation,))


def _declaration(annotation):
    origin = typing.get_origin(annotation)
    args = typing.get_args(annotation)
    if origin is Required:
        return _Declaration('required', Operation if _is_operation_type(args[0]) else args[0], annotation)
    if origin is Const:
        return _Declaration('const', args[0], annotation)
    if origin is Param:
        return _Declaration('param', args[0], annotation)
    if annotation is Clock:
        return _Declaration('clock', Clock, annotation)
    if annotation is Input or origin is Input:
        return _Declaration('input', args[0] if args else Any, annotation)
    if annotation is Output or origin is Output:
        return _Declaration('output', args[0] if args else Any, annotation)
    if _is_operation_type(annotation):
        return _Declaration('operation', Operation, annotation)
    if annotation is collections.abc.Callable or origin is collections.abc.Callable:
        return _Declaration('function', annotation, annotation)
    return _Declaration('var', annotation, annotation)


# Names used by the framework, which cannot be used for declarations.
_RESERVED = {'entry', 'exit', 'transitions', 'initial', 'during',
             'logger', 'timer_period', 'time_unit', 'execute', 'configuration', 'finished', 'max_transitions', 'on_change', 'wait'}


# ---------------------------------------------------------------------------------
# Scopes: robotic platforms, controllers, state machines and operations
# ---------------------------------------------------------------------------------

class Scope:
    '''
    Declarations of variables, constants, clocks, events and operations, as class annotations.
    It can be used directly to define a robotic platform or controller, and is the base of
    StateMachine and Operation.

        class Platform(Scope):
            cmd_vel: Callable[[TwistStamped], None] # a platform operation, given at construction
            obstacle: Input[Direction]              # an input event, set by a ROS callback

        class Controller(Scope):
            move: type[Operation] = Move            # an operation defined by a state machine

    Reading or writing a name that is not declared is an error.
    '''

    _declarations: dict = {}

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        declarations = {}
        for base in reversed(cls.__bases__):
            declarations.update(getattr(base, '_declarations', {}))
        try:
            annotations = inspect.get_annotations(cls, eval_str = True)
        except NameError as e:
            raise TypeError(f"In {cls.__qualname__}: {e}") from None
        for name, annotation in annotations.items():
            if name.startswith('_'):
                continue
            declaration = _declaration(annotation)
            if name in cls.__dict__:
                # The value in the class is the initial value: values are kept in instances.
                declaration.has_default = True
                declaration.default = cls.__dict__[name]
                delattr(cls, name)
            if name in _RESERVED or hasattr(cls, name):
                raise TypeError(f"In {cls.__qualname__}: '{name}' is declared, but is also the name of a state or method.")
            declarations[name] = declaration
        cls._declarations = declarations

    def __init__(self, parent = None, **values):
        object.__setattr__(self, '_parent', parent)
        object.__setattr__(self, '_values', {})
        missing = []
        for name, declaration in self._declarations.items():
            if declaration.kind == 'required':
                if name in values:
                    raise TypeError(f"{self._name()}: '{name}' is required, so it is provided by an enclosing scope, not at construction.")
                continue
            if name in values:
                value = values.pop(name)
            elif declaration.has_default:
                value = declaration.default
                if declaration.kind == 'input':
                    value = copy.deepcopy(value)
            elif declaration.kind == 'clock':
                value = Clock()
            elif declaration.kind == 'input':
                value = Input()
            else:
                missing.append(name)
                continue
            self._values[name] = self._initial_value(name, declaration, value)
        if missing:
            raise TypeError(f"{self._name()}: no value given for {', '.join(map(repr, missing))}.")
        if values:
            raise TypeError(f"{self._name()}: {', '.join(map(repr, values))} not declared.")

    def _name(self):
        return type(self).__qualname__

    def _initial_value(self, name, declaration, value):
        kind = declaration.kind
        if kind in ('var', 'const', 'param'):
            # As in RoboChart, values are copied, so variables never share (mutable) objects.
            return copy.deepcopy(declaration.validate(name, value))
        if kind == 'clock' and not isinstance(value, Clock):
            raise TypeError(f"{self._name()}: '{name}' is a clock.")
        if kind == 'input':
            if not isinstance(value, Input):
                raise TypeError(f"{self._name()}: '{name}' is an input event.")
            if declaration.type is not Any:
                value._validate = lambda v: declaration.validate(name, v)
        if kind == 'output' and not isinstance(value, Output):
            if not callable(value):
                raise TypeError(f"{self._name()}: '{name}' is an output event, so it needs a function that implements it.")
            value = Output(value)
        if kind == 'function' and not callable(value):
            raise TypeError(f"{self._name()}: '{name}' must be a function.")
        if kind == 'operation' and not (isinstance(value, type) and issubclass(value, Operation)):
            raise TypeError(f"{self._name()}: '{name}' must be a class that inherits from Operation.")
        return value

    # --- Resolution of names ---

    def _find(self, name):
        ''' The scope that declares 'name', searching this scope and its enclosing scopes. '''
        scope = self
        while scope is not None:
            declaration = scope._declarations.get(name)
            if declaration is not None and declaration.kind != 'required':
                return scope, declaration
            scope = scope._parent
        return None

    def _outer(self, name):
        ''' The scope that provides a name required by this scope. '''
        return self._parent._find(name) if self._parent is not None else None

    def _provider(self, name):
        ''' The scope that provides 'name', which must be declared in this scope. '''
        declaration = self._declarations.get(name)
        if declaration is None:
            return None
        if declaration.kind == 'required':
            return self._outer(name)
        return self, declaration

    def _resolve(self, name):
        if name not in self._declarations:
            raise AttributeError(f"'{name}' is not declared in {self._name()}. Declare it, or declare it as Required[...] if it is provided by an enclosing scope.")
        found = self._provider(name)
        if found is None:
            raise AttributeError(f"'{name}' is required by {self._name()}, but is not provided.")
        return found

    def __getattr__(self, name) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        return self._lookup(name, self)

    def _lookup(self, name, origin):
        scope, declaration = self._resolve(name)
        value = scope._values[name]
        if declaration.kind == 'operation':
            # Calls of the operation are bound to the scope where the lookup started.
            return _OperationRef(value, origin)
        return value

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
            return
        scope, declaration = self._resolve(name)
        if declaration.kind not in ('var', 'param'):
            raise AttributeError(f"'{name}' is declared as {declaration.kind} in {scope._name()}, so it cannot be assigned.")
        # As in RoboChart, assignment copies the value: after 'self.v = self.w', changing v does not change w.
        scope._values[name] = copy.deepcopy(declaration.validate(name, value))

    def _scopes(self):
        scope = self
        while scope is not None:
            yield scope
            scope = scope._parent


# ---------------------------------------------------------------------------------
# Checking that required names are provided, at construction of a state machine
# ---------------------------------------------------------------------------------

def _with_value(found, name):
    ''' (declaration, value) for the result of a search for 'name'. '''
    return None if found is None else (found[1], found[0]._values[name])


def _class_provider(cls, provide_outer):
    ''' Provider of the names declared in an operation 'cls', which is not instantiated. '''
    def provide(name):
        declaration = cls._declarations.get(name)
        if declaration is None:
            return None
        if declaration.kind == 'required':
            return provide_outer(name)
        return declaration, declaration.default if declaration.has_default else None
    return provide


def _check_requirements(cls, provide_outer, provide_inner, where, problems, checked):
    '''
    Check that the names required by 'cls' are provided by 'provide_outer', with the declared
    types, and, recursively, that the operations it can call have their requirements satisfied.
    '''
    for name, declaration in cls._declarations.items():
        if declaration.kind != 'required':
            continue
        found = provide_outer(name)
        if found is None:
            problems.append(f"{cls.__qualname__} requires '{name}', which is not provided by {where}.")
        elif declaration.type is Operation:
            if found[0].kind != 'operation':
                problems.append(f"{cls.__qualname__} requires '{name}' to be an operation, but it is provided as {found[0].kind}.")
        elif declaration.type is not Any and found[0].annotation != declaration.type:
            problems.append(f"{cls.__qualname__} requires '{name}' of type {_type_name(declaration.type)}, "
                            f"but it is provided with type {_type_name(found[0].annotation)}.")
    for name in cls._declarations:
        found = provide_inner(name)
        if found is not None and found[0].kind == 'operation' and found[1] is not None:
            operation = found[1]
            if (operation, cls) not in checked:
                checked.add((operation, cls))
                _check_requirements(operation, provide_inner, _class_provider(operation, provide_inner),
                                    f"{cls.__qualname__}, which calls it", problems, checked)


# ---------------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------------

class _StateMeta(type):
    ''' Accessing a State class through an instance, as in 'self.Forward', gives a target for a transition. '''

    def __get__(cls, parent, owner = None):
        return cls if parent is None else _StateRef(cls, parent)


class _StateRef:
    ''' A state, as the target of a transition. '''

    def __init__(self, cls, parent):
        self.cls = cls
        self.parent = parent

    def __repr__(self):
        return self.cls.__name__


_STATE_METHODS = ('entry', 'during', 'transitions', 'exit', 'initial')


class State(metaclass = _StateMeta):
    '''
    A state, defined by any of the methods:

      - entry(self): the entry action;
      - during(self): the during action, which starts after the entry action (and the entry of
        substates), and is interrupted when a transition of this state, or of an enclosing
        state, is taken. If it finishes, it is not restarted;
      - transitions(self): called on every cycle; returns the target of an enabled transition
        (a state, as in 'self.Turning', FINAL, or a junction), or (target, action), where the
        action is executed after this state has been exited, or None if no transition is enabled.
        Alternatively, it may be a generator that yields when no transition is enabled;
      - exit(self): the exit action;
      - initial(self): the initial junction of a composite state, whose substates are its
        nested State classes.

    Entry, during and exit actions, and actions of transitions, may be generators, taking
    several cycles. While a during action takes several cycles, the transitions of this state,
    and of enclosing states, are checked on every cycle, so it can be interrupted.
    Inside a state, 'self' gives access to what is declared in the machine (or operation), and
    to the other states. Actions of transitions are functions, typically defined in 'transitions'
    next to the transition, and returned without calling them, as in 'return self.Driving, action'.

    A junction is also a function defined in 'transitions', marked with @junction, which returns
    the result of one of its transitions, as 'transitions' does. It is evaluated after the action
    of the transition leading to it, so its guards see the effect of that action:

        def transitions(self):
            if self.obstacle.triggered():
                self.obs = self.obstacle.take()             # trigger obstacle?obs

                @junction
                def j0():
                    if self.obs == Direction.LEFT:          # guard
                        def action():
                            yield from self.move(0.0, avel)
                        return self.Rotating, action
                    ...

                return j0, self.C.reset                     # action #C, to the junction

    If no transition out of a junction is enabled (it returns None), the machine waits at the
    junction, and the junction is evaluated again on every cycle. As the machine is in the
    middle of a transition, it is busy while it waits: enclosing states cannot interrupt it.
    '''

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        for name, value in cls.__dict__.items():
            if callable(value) and not isinstance(value, type) and not name.startswith('_') and name not in _STATE_METHODS:
                close = difflib.get_close_matches(name, _STATE_METHODS, n = 1, cutoff = 0.75)
                if close:
                    raise TypeError(f"State {cls.__qualname__} defines '{name}': did you mean '{close[0]}'?")

    def __init__(self, parent):
        object.__setattr__(self, '_parent', parent)
        object.__setattr__(self, '_region', None)

    def __getattr__(self, name) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(self._parent, name)

    def __setattr__(self, name, value):
        # States have no variables of their own: assign in the machine (or operation).
        setattr(self._parent, name, value)

    def entry(self):
        return None

    def during(self):
        return None

    def transitions(self):
        return None

    def exit(self):
        return None

    def _name(self):
        return type(self).__name__

    def _machine(self):
        parent = self._parent
        while isinstance(parent, State):
            parent = parent._parent
        return parent._root()

    def _check(self):
        ''' Generator for the transitions, which returns the result of the transition taken. '''
        result = self.transitions()
        # Whether each step of this generator evaluates the transitions afresh.
        object.__setattr__(self, '_evaluated', not _is_generator(result))
        if _is_generator(result):
            target = yield from result
            if target is None:
                raise TypeError(f"transitions() of state {self._name()} finished without returning a target.")
            return target
        while result is None:
            yield
            result = self.transitions()
        return result

    def _run(self):
        ''' Generator for the execution of this state, stepped once per cycle. '''

        # --- Entry action ---
        effect = self.entry()
        if _is_generator(effect):
            for _ in effect:
                yield MachineStatus.BUSY

        # --- Enter substates ---
        machine = self._machine()
        region = None
        if hasattr(type(self), 'initial'):
            region = _Region(self.initial, self._name(), machine)
            object.__setattr__(self, '_region', region)
            for _ in region.enter():
                yield MachineStatus.BUSY

        # --- Transitions of this state have priority over those of its substates ---
        body = self._check()
        during = None
        status = None
        exiting = None
        try:
            next(body)
            # --- During action, after the first check of the transitions ---
            effect = self.during()
            if _is_generator(effect):
                during = _advance(effect)
            while True:
                yield status
                # Transitions are not checked while a substate is entering or exiting, or a transition action is executing.
                if status is not MachineStatus.BUSY:
                    next(body)
                # No transition has been taken, so the during action, if not finished, continues.
                if during is not None:
                    during = _advance(during)
                if region is not None:
                    entered = machine._entered
                    status = MachineStatus.BUSY if region.step() is MachineStatus.BUSY else None
                    # A substate took a transition, which may have enabled a transition of this
                    # state at the same instant, so check again. (A 'transitions' written as a
                    # generator is not re-evaluated in the same cycle, but in the next.)
                    if status is not MachineStatus.BUSY and machine._entered != entered and self._evaluated:
                        next(body)
        except StopIteration as e:
            # A transition of this state is taken.
            result = e.value
        except _Exiting as e:
            # An enclosing state is being exited.
            exiting = e
        # A transition interrupts the during action, if not finished.
        if during is not None:
            during.close()
        body.close()

        # --- Exit substates, then this state ---
        if region is not None:
            for _ in region.exit():
                yield MachineStatus.BUSY
            object.__setattr__(self, '_region', None)
        effect = self.exit()
        if _is_generator(effect):
            for _ in effect:
                yield MachineStatus.BUSY

        if exiting is not None:
            raise exiting
        return result


def junction(function):
    '''
    Decorator that marks a function as a junction: it returns the result of one of its
    transitions, as 'transitions' does, or None if none is enabled. Marking junctions makes
    mistakes, like returning an action without its target, errors rather than junctions.
    '''
    function._robochart_junction = True
    return function


def _is_junction(target):
    return callable(target) and getattr(target, '_robochart_junction', False)


def _describe(target):
    if isinstance(target, _StateRef) or target is FINAL:
        return repr(target)
    return getattr(target, '__name__', repr(target))


def _is_generator(value):
    return isinstance(value, (types.GeneratorType, _Call))


def _wait(machine, units):
    ''' Generator for wait(units): it finishes once 'units' whole time units have passed. '''
    if not isinstance(units, int) or isinstance(units, bool) or units < 0:
        raise ValueError(f"wait takes a number of time units, a natural number, but got {units!r}.")
    # A clock of its own, stepped by the machine only while waiting.
    clock = Clock()
    machine._register([clock])
    try:
        while clock.since() < units:
            yield
    finally:
        machine._unregister([clock])


def _advance(during):
    ''' Step a during action by one cycle, returning None if it has finished. '''
    try:
        next(during)
        return during
    except StopIteration:
        return None


def _at_junction(junction):
    '''
    Wait at a junction till one of its transitions is enabled: the junction is evaluated again
    on every cycle. If none can ever be enabled, the machine is deadlocked here. Unlike in a
    state, the machine is in the middle of a transition, so it is busy: enclosing states
    cannot interrupt it.
    '''
    while True:
        yield MachineStatus.BUSY
        result = junction()
        if result is not None:
            return result


def _then(effect, target):
    ''' Execute the action of a transition, possibly over several cycles, then go to 'target'. '''
    for _ in effect:
        yield MachineStatus.BUSY
    return target


class _Region:
    ''' Executes the states of a machine, operation or composite state, from its initial junction. '''

    def __init__(self, initial, name, machine):
        self._initial = initial
        self._name = name
        self._machine = machine
        self.node = None
        self.state = None
        self.finished = False
        # Description of the transition whose action is executing, if any, and its source.
        self.transition = None
        self.source = None

    def _next(self, result, source = 'initial junction'):
        '''
        The next node, given the result of a transition: a target, or (target, action).
        If the target is a junction, it is evaluated after the action, to take one of its transitions.
        '''
        if isinstance(result, tuple) and len(result) == 2:
            target, action = result
            if callable(target) and (action is FINAL or isinstance(action, _StateRef) or _is_junction(action)):
                raise TypeError(f"In {self._name}: a transition was given as ({_describe(target)}, "
                                f"{_describe(action)}): did you mean ({_describe(action)}, {_describe(target)}), "
                                f"with the target first, then the action?")
            if not callable(action):
                raise TypeError(f"In {self._name}: the action of a transition must be a function, "
                                f"called after the source state is exited, but got {action!r}.")
        else:
            target, action = result, None
        self._check_target(target)
        effect = action() if action is not None else None
        if _is_generator(effect):
            self.source = source
            self.transition = f"{source} -> {_describe(target)}"
            return _then(effect, target)
        if _is_junction(target):
            result = target()
            if result is None:
                # No transition out of the junction is enabled: wait there.
                self.source = source
                self.transition = f"{source} -> {_describe(target)}"
                return _at_junction(target)
            return self._next(result, source)
        return target

    def _check_target(self, target):
        if target is FINAL or isinstance(target, _StateRef) or _is_junction(target):
            return
        hint = ""
        if callable(target):
            hint = (f" If '{_describe(target)}' is the action of the transition, return it with the target, "
                    f"as in '(self.Forward, {_describe(target)})'; if it is a junction, mark it with @junction.")
        raise TypeError(f"In {self._name}: a transition must lead to a state (as in 'self.Forward'), FINAL, "
                        f"or a junction, possibly with an action, as in '(self.Forward, action)', but got "
                        f"{target!r}.{hint}")

    def step(self):
        ''' Execute one cycle. '''
        if self.finished:
            return MachineStatus.FINISHED
        if self.node is None:
            self.node = self._next(self._initial())
        while True:
            try:
                if self.node is FINAL:
                    # A final state has no outgoing transitions: stay there.
                    self.node = None
                    self.finished = True
                    return MachineStatus.FINISHED
                if isinstance(self.node, _StateRef):
                    self._machine._entering(self.node)
                    self.transition = None
                    self.source = None
                    self.state = self.node.cls(self.node.parent)
                    self.node = self.state._run()
                return next(self.node)
            except StopIteration as e:
                # The state has been exited, or the action of a transition has finished.
                source = self.state._name() if self.state is not None else (self.source or 'initial junction')
                self.state = None
                self.node = self._next(e.value, source)

    def enter(self):
        ''' Generator that enters this region, yielding while that takes several cycles. '''
        self.node = None
        self.finished = False
        while self.step() is MachineStatus.BUSY:
            yield MachineStatus.BUSY

    def exit(self):
        ''' Generator that exits the active state of this region, yielding while that takes several cycles. '''
        node = self.node
        self.node = None
        self.state = None
        self.finished = False
        self.transition = None
        if _is_generator(node):
            try:
                node.throw(_Exiting())
                while True:
                    yield MachineStatus.BUSY
                    next(node)
            except (_Exiting, StopIteration):
                pass

    def configuration(self):
        if self.state is None:
            if self.transition is not None:
                return [self.transition]
            return ['Final'] if self.finished else []
        inner = self.state._region.configuration() if self.state._region is not None else []
        return [self.state._name()] + inner


# ---------------------------------------------------------------------------------
# State machines and operations
# ---------------------------------------------------------------------------------

class StateMachine(Scope):
    '''
    A state machine. Its initial junction is the method 'initial', which returns the
    initial state, and its states are nested State classes.

    'parent' is the scope providing the names that the machine declares as Required, for
    example, a robotic platform. Values of constants, and initial values of variables, can
    be given as keyword arguments. Construction fails if a required name is not provided,
    including those required by the operations the machine calls.

    'timer_period' is the time between timed cycles (ticks), for example, the period of a ROS
    timer, and 'time_unit' is the time that corresponds to one RoboChart time unit, which must
    be a whole number of timer periods. Clocks count whole time units.

    'max_transitions' is the number of states that can be entered in one cycle, beyond
    which there is likely a loop of transitions that never lets time pass.

    'on_change', if given, is called with the new configuration (see 'configuration') at the
    end of every cycle in which it changed, for example, to publish it on a ROS topic. It is
    called before the next cycle can start, so, also with several threads, it sees the
    changes in order.

    'execute' can be called from different threads, for example, by callbacks of a ROS
    MultiThreadedExecutor: cycles are executed one at a time, and input events used by the
    machine are not set during a cycle, but before or after it.
    '''

    def __init__(self, logger, timer_period, time_unit = 1, parent = None, max_transitions = 10_000,
                 on_change = None, **values):
        super().__init__(parent, **values)
        object.__setattr__(self, 'logger', logger)
        object.__setattr__(self, 'timer_period', timer_period)
        object.__setattr__(self, 'time_unit', time_unit)
        object.__setattr__(self, 'max_transitions', max_transitions)
        object.__setattr__(self, 'on_change', on_change)
        object.__setattr__(self, '_last_configuration', [])

        # A time unit must be a whole number of ticks (timed cycles), so that clocks count whole time units.
        ticks = time_unit / timer_period
        if round(ticks) < 1 or abs(ticks - round(ticks)) > 1e-9 * ticks:
            raise ValueError(f"{self._name()}: time_unit ({time_unit}) must be a whole number of timer periods "
                             f"({timer_period}), so that since(C) counts whole time units.")
        object.__setattr__(self, '_ticks_per_unit', round(ticks))

        problems = []
        _check_requirements(type(self),
                            lambda name: None if parent is None else _with_value(parent._find(name), name),
                            lambda name: _with_value(self._provider(name), name),
                            'the parent scope' if parent is None else type(parent).__qualname__,
                            problems, set())
        if problems:
            raise TypeError("\n".join(problems))

        object.__setattr__(self, '_clocks', [])
        self._register([value for value in self._values.values() if isinstance(value, Clock)])
        # Number of states entered, in total, and at the start of the current cycle.
        object.__setattr__(self, '_entered', 0)
        object.__setattr__(self, '_cycle_start', 0)

        # Cycles are executed one at a time, also when 'execute' is called from different threads.
        object.__setattr__(self, '_lock', threading.RLock())
        object.__setattr__(self, '_executing', False)

        # Input events of this machine and its enclosing scopes, which are set outside cycles.
        object.__setattr__(self, '_inputs', [value for scope in self._scopes()
                                             for value in scope._values.values() if isinstance(value, Input)])
        for event in self._inputs:
            if event._lock is None:
                event._lock = self._lock
        object.__setattr__(self, '_region', _Region(self.initial, self._name(), self))

    def _root(self):
        return self

    def wait(self, units):
        '''
        The RoboChart action wait(units), as in 'yield from self.wait(3)': it waits for 'units'
        whole time units, counted, like clocks, on timed cycles.
        '''
        return _wait(self._root(), units)

    def _entering(self, state):
        object.__setattr__(self, '_entered', self._entered + 1)
        if self._entered - self._cycle_start > self.max_transitions:
            raise RuntimeError(f"{self._name()}: more than {self.max_transitions} states entered in one cycle, "
                               f"the last of which is {state!r}. There is probably a loop of transitions "
                               f"that never lets time pass.")

    def _register(self, clocks):
        for clock in clocks:
            clock.ticks_per_unit = self._ticks_per_unit
            self._clocks.append(clock)

    def _unregister(self, clocks):
        self._clocks[:] = [clock for clock in self._clocks if all(clock is not c for c in clocks)]

    def initial(self):
        raise NotImplementedError(f"Initial junction of machine '{self._name()}' has not been implemented.")

    def execute(self, timed = True):
        '''
        Execute one cycle of the machine: as far as possible, till every active state is waiting
        for a transition to be enabled, or for an action that takes several cycles. If 'timed',
        for example when called from a ROS timer, the clocks are stepped first. Input events not
        consumed in this cycle are then handled as given by their 'keep' option.
        Returns MachineStatus.BUSY or FINISHED, or None.
        '''
        with self._lock:
            if self._executing:
                raise RuntimeError(f"{self._name()}: execute() was called during a cycle, for example, from one of its actions.")
            object.__setattr__(self, '_executing', True)
            object.__setattr__(self, '_cycle_start', self._entered)
            try:
                if timed:
                    for clock in self._clocks:
                        clock.step()
                status = self._region.step()
                configuration = self.configuration()
                if configuration != self._last_configuration:
                    object.__setattr__(self, '_last_configuration', configuration)
                    if self.on_change is not None:
                        # Still part of this cycle: calling execute() from on_change is an error.
                        self.on_change(list(configuration))
                return status
            finally:
                object.__setattr__(self, '_executing', False)
                for event in self._inputs:
                    event._end_of_cycle()

    @property
    def finished(self):
        return self._region.finished

    def configuration(self):
        '''
        The names of the active states, from the outermost, for example ['Driving', 'Forward'].
        While the action of a transition executes, it is described instead, as in 'Rotating -> Driving'.
        '''
        return self._region.configuration()


class Operation(Scope):
    '''
    A RoboChart operation defined by a state machine. Its initial junction is the method
    'initial', and its states are nested State classes. It terminates when it reaches FINAL.

    Its parameters are declared as Param[...], and are given, in order, as the arguments of
    a call. Its other variables are reset on each call. Names declared as Required are
    provided by the caller: the machine or operation calling it, which must declare them.

    It is called from an action, as in 'yield from self.move(0.0, avel)'.
    '''

    def __init__(self, caller, *args, **kwargs):
        parameters = [name for name, declaration in self._declarations.items() if declaration.kind == 'param']
        if len(args) > len(parameters):
            raise TypeError(f"{self._name()} takes {len(parameters)} arguments, but {len(args)} were given.")
        values = dict(zip(parameters, args))
        for name in kwargs:
            if name in values:
                raise TypeError(f"{self._name()} got multiple values for '{name}'.")
        values.update(kwargs)
        super().__init__(caller, **values)
        root = caller._root()
        object.__setattr__(self, '_machine', root)
        object.__setattr__(self, 'logger', root.logger)
        object.__setattr__(self, 'timer_period', root.timer_period)
        object.__setattr__(self, 'time_unit', root.time_unit)

    def _root(self):
        return self._machine

    def wait(self, units):
        '''
        The RoboChart action wait(units), as in 'yield from self.wait(3)': it waits for 'units'
        whole time units, counted, like clocks, on timed cycles.
        '''
        return _wait(self._root(), units)

    def _outer(self, name):
        # Required names are provided by the caller, which must declare them.
        return self._parent._provider(name)

    def initial(self):
        raise NotImplementedError(f"Initial junction of operation '{self._name()}' has not been implemented.")

    def _execute(self):
        clocks = [value for value in self._values.values() if isinstance(value, Clock)]
        self._machine._register(clocks)
        try:
            region = _Region(self.initial, self._name(), self._machine)
            while True:
                status = region.step()
                if status is MachineStatus.FINISHED:
                    return
                yield status
        finally:
            self._machine._unregister(clocks)


class _OperationRef:
    ''' An operation, bound to the scope that calls it. '''

    def __init__(self, operation, caller):
        self.operation = operation
        self.caller = caller

    def __call__(self, *args, **kwargs):
        return _Call(self.operation(self.caller, *args, **kwargs)._execute(), self.operation.__qualname__)

    def __repr__(self):
        return f"{self.operation.__qualname__} called from {self.caller._name()}"


class _Call:
    ''' A call of an operation: a generator, which warns if it is never executed. '''

    def __init__(self, generator, name):
        self._generator = generator
        self._name = name
        self._started = False

    def __iter__(self):
        return self

    def __next__(self):
        self._started = True
        return next(self._generator)

    def send(self, value):
        self._started = True
        return self._generator.send(value)

    def throw(self, exception):
        self._started = True
        return self._generator.throw(exception)

    def close(self):
        self._started = True
        self._generator.close()

    def __del__(self):
        if not self._started:
            warnings.warn(f"Operation {self._name} was called but never executed: did you forget 'yield from'?",
                          RuntimeWarning, stacklevel = 2)
