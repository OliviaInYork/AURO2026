
from typing import TypeVar, Set

T = TypeVar("T")

def union(a:Set[T], b:Set[T]):
    return a | b

# The RoboChart set {a..b}: the integers from a to b, including b.
def interval(a:int, b:int) -> Set[int]:
    return set(range(a, b + 1))
