"""Core data models for the robotic sorting system."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class Color(Enum):
    """Object color enumeration."""
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    YELLOW = "yellow"


class Size(Enum):
    """Object size enumeration."""
    SMALL = "small"
    MEDIUM = "medium"
    LARGE = "large"


class ObjectType(Enum):
    """Object type enumeration."""
    TYPE_A = "type_a"
    TYPE_B = "type_b"
    TYPE_C = "type_c"


class SortingCriteria(Enum):
    """Sorting criteria enumeration."""
    BY_COLOR = "by_color"
    BY_SIZE = "by_size"
    BY_TYPE = "by_type"


class ObjectState(Enum):
    """Object state enumeration."""
    IN_SOURCE = "in_source"
    HELD = "held"
    IN_ZONE = "in_zone"


class OperationType(Enum):
    """Operation type enumeration."""
    PICK = "pick"
    PLACE = "place"


class ErrorType(Enum):
    """Error type enumeration."""
    MISSED_PICK = "missed_pick"
    COLLISION = "collision"
    ZONE_FULL = "zone_full"
    OBJECT_NOT_FOUND = "object_not_found"
    INVALID_STATE = "invalid_state"
    MAX_RETRIES_EXCEEDED = "max_retries_exceeded"


@dataclass
class Position2D:
    """2D position in the workspace."""
    x: float
    y: float

    def distance_to(self, other: 'Position2D') -> float:
        """Calculate Euclidean distance to another position."""
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5


@dataclass
class BoundingBox:
    """Rectangular bounding box defined by min and max corners."""
    min: Position2D
    max: Position2D

    def contains(self, point: Position2D) -> bool:
        """Check if a point is within the bounding box."""
        return (self.min.x <= point.x <= self.max.x and
                self.min.y <= point.y <= self.max.y)

    def intersects(self, other: 'BoundingBox') -> bool:
        """Check if this bounding box intersects with another."""
        return not (self.max.x < other.min.x or
                    self.min.x > other.max.x or
                    self.max.y < other.min.y or
                    self.min.y > other.max.y)


@dataclass
class SourceArea:
    """Source area where objects are initially placed."""
    bounds: BoundingBox
    capacity: int


@dataclass
class TargetZone:
    """Target zone for sorted objects."""
    id: str
    bounds: BoundingBox
    capacity: int
    criteria_value: str  # e.g., "red", "large", "type_a"
    current_count: int = 0

    def is_full(self) -> bool:
        """Check if the zone is at capacity."""
        return self.current_count >= self.capacity

    def add_object(self) -> bool:
        """Add an object to the zone if not full."""
        if self.is_full():
            return False
        self.current_count += 1
        return True

    def remove_object(self) -> bool:
        """Remove an object from the zone."""
        if self.current_count <= 0:
            return False
        self.current_count -= 1
        return True


@dataclass
class Object:
    """Object with properties that can be sorted."""
    id: str
    color: Color
    size: Size
    type: ObjectType
    position: Position2D
    state: ObjectState = ObjectState.IN_SOURCE


@dataclass
class OperationResult:
    """Result of a pick or place operation."""
    success: bool
    operation_type: OperationType
    object_id: Optional[str] = None
    error: Optional[ErrorType] = None
    retry_count: int = 0
    timestamp: float = 0.0


@dataclass
class MovementResult:
    """Result of a movement operation."""
    success: bool
    error: Optional[ErrorType] = None
    final_position: Position2D = None
