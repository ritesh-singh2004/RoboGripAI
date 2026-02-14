"""Unit tests for ObjectManager component."""

import pytest
from robotic_sorting.models import (
    SourceArea, TargetZone, BoundingBox, Position2D,
    Color, Size, ObjectType, ObjectState, SortingCriteria
)
from robotic_sorting.object_manager import ObjectManager, ObjectSpec


@pytest.fixture
def source_area():
    """Create a test source area."""
    return SourceArea(
        bounds=BoundingBox(
            min=Position2D(0, 0),
            max=Position2D(10, 10)
        ),
        capacity=20
    )


@pytest.fixture
def target_zones():
    """Create test target zones."""
    return [
        TargetZone(
            id="red_zone",
            bounds=BoundingBox(min=Position2D(20, 0), max=Position2D(30, 10)),
            capacity=5,
            criteria_value="red"
        ),
        TargetZone(
            id="blue_zone",
            bounds=BoundingBox(min=Position2D(20, 15), max=Position2D(30, 25)),
            capacity=5,
            criteria_value="blue"
        )
    ]


@pytest.fixture
def object_manager(source_area, target_zones):
    """Create a test object manager."""
    return ObjectManager(source_area, target_zones)


def test_create_objects_generates_unique_ids(object_manager):
    """Test that create_objects generates unique IDs for each object."""
    specs = [
        ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A),
        ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B),
        ObjectSpec(Color.GREEN, Size.LARGE, ObjectType.TYPE_C)
    ]
    
    ids = object_manager.create_objects(specs)
    
    assert len(ids) == 3
    assert len(set(ids)) == 3  # All IDs are unique


def test_create_objects_places_in_source(object_manager):
    """Test that created objects are placed in source area."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    
    ids = object_manager.create_objects(specs)
    obj = object_manager.get_object(ids[0])
    
    assert obj is not None
    assert obj.state == ObjectState.IN_SOURCE
    assert object_manager.source_area.bounds.contains(obj.position)


def test_create_objects_with_custom_position(object_manager):
    """Test creating objects with custom initial positions."""
    custom_pos = Position2D(5, 5)
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, custom_pos)]
    
    ids = object_manager.create_objects(specs)
    obj = object_manager.get_object(ids[0])
    
    assert obj.position.x == custom_pos.x
    assert obj.position.y == custom_pos.y


def test_get_available_objects_returns_only_source_objects(object_manager):
    """Test that get_available_objects returns only objects in source."""
    specs = [
        ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A),
        ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B)
    ]
    ids = object_manager.create_objects(specs)
    
    # Initially all objects should be available
    available = object_manager.get_available_objects()
    assert len(available) == 2
    
    # Remove one from source
    object_manager.remove_from_source(ids[0])
    available = object_manager.get_available_objects()
    assert len(available) == 1
    assert available[0].id == ids[1]


def test_remove_from_source_changes_state(object_manager):
    """Test that remove_from_source changes object state to HELD."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = object_manager.create_objects(specs)
    
    result = object_manager.remove_from_source(ids[0])
    obj = object_manager.get_object(ids[0])
    
    assert result is True
    assert obj.state == ObjectState.HELD


def test_remove_from_source_fails_for_nonexistent_object(object_manager):
    """Test that remove_from_source fails for non-existent object."""
    result = object_manager.remove_from_source("nonexistent_id")
    assert result is False


def test_remove_from_source_fails_for_already_held_object(object_manager):
    """Test that remove_from_source fails if object already held."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = object_manager.create_objects(specs)
    
    # Remove once (should succeed)
    assert object_manager.remove_from_source(ids[0]) is True
    
    # Try to remove again (should fail)
    assert object_manager.remove_from_source(ids[0]) is False



def test_place_in_zone_success(object_manager, target_zones):
    """Test successful placement of object in zone."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = object_manager.create_objects(specs)
    
    # Remove from source first
    object_manager.remove_from_source(ids[0])
    
    # Place in zone
    result = object_manager.place_in_zone(ids[0], target_zones[0])
    obj = object_manager.get_object(ids[0])
    
    assert result is True
    assert obj.state == ObjectState.IN_ZONE
    assert target_zones[0].bounds.contains(obj.position)
    assert target_zones[0].current_count == 1


def test_place_in_zone_fails_if_not_held(object_manager, target_zones):
    """Test that place_in_zone fails if object not in HELD state."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = object_manager.create_objects(specs)
    
    # Try to place without removing from source first
    result = object_manager.place_in_zone(ids[0], target_zones[0])
    
    assert result is False


def test_place_in_zone_fails_if_zone_full(object_manager, target_zones):
    """Test that place_in_zone fails if zone is at capacity."""
    # Create objects equal to zone capacity
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A) 
             for _ in range(6)]
    ids = object_manager.create_objects(specs)
    
    zone = target_zones[0]  # capacity = 5
    
    # Fill the zone
    for i in range(5):
        object_manager.remove_from_source(ids[i])
        result = object_manager.place_in_zone(ids[i], zone)
        assert result is True
    
    # Try to place one more (should fail)
    object_manager.remove_from_source(ids[5])
    result = object_manager.place_in_zone(ids[5], zone)
    
    assert result is False
    assert zone.current_count == 5


def test_return_to_source_success(object_manager):
    """Test successful return of object to source area."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = object_manager.create_objects(specs)
    
    # Remove from source
    object_manager.remove_from_source(ids[0])
    obj = object_manager.get_object(ids[0])
    assert obj.state == ObjectState.HELD
    
    # Return to source
    result = object_manager.return_to_source(ids[0])
    
    assert result is True
    assert obj.state == ObjectState.IN_SOURCE
    assert object_manager.source_area.bounds.contains(obj.position)


def test_return_to_source_fails_if_not_held(object_manager):
    """Test that return_to_source fails if object not in HELD state."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = object_manager.create_objects(specs)
    
    # Try to return without removing from source first
    result = object_manager.return_to_source(ids[0])
    
    assert result is False


def test_return_to_source_fails_for_nonexistent_object(object_manager):
    """Test that return_to_source fails for non-existent object."""
    result = object_manager.return_to_source("nonexistent_id")
    assert result is False


def test_get_object_returns_none_for_nonexistent(object_manager):
    """Test that get_object returns None for non-existent ID."""
    obj = object_manager.get_object("nonexistent_id")
    assert obj is None


def test_empty_source_area(object_manager):
    """Test behavior with empty source area."""
    available = object_manager.get_available_objects()
    assert len(available) == 0


# Tests for sorting logic (Task 2.2)

def test_get_target_zone_by_color(object_manager, target_zones):
    """Test get_target_zone with color sorting criteria."""
    specs = [
        ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A),
        ObjectSpec(Color.BLUE, Size.LARGE, ObjectType.TYPE_B)
    ]
    ids = object_manager.create_objects(specs)
    
    red_obj = object_manager.get_object(ids[0])
    blue_obj = object_manager.get_object(ids[1])
    
    red_zone = object_manager.get_target_zone(red_obj, SortingCriteria.BY_COLOR)
    blue_zone = object_manager.get_target_zone(blue_obj, SortingCriteria.BY_COLOR)
    
    assert red_zone is not None
    assert red_zone.id == "red_zone"
    assert blue_zone is not None
    assert blue_zone.id == "blue_zone"


def test_get_target_zone_by_size():
    """Test get_target_zone with size sorting criteria."""
    source = SourceArea(
        bounds=BoundingBox(min=Position2D(0, 0), max=Position2D(10, 10)),
        capacity=20
    )
    zones = [
        TargetZone(
            id="small_zone",
            bounds=BoundingBox(min=Position2D(20, 0), max=Position2D(30, 10)),
            capacity=5,
            criteria_value="small"
        ),
        TargetZone(
            id="large_zone",
            bounds=BoundingBox(min=Position2D(20, 15), max=Position2D(30, 25)),
            capacity=5,
            criteria_value="large"
        )
    ]
    manager = ObjectManager(source, zones)
    
    specs = [
        ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A),
        ObjectSpec(Color.BLUE, Size.LARGE, ObjectType.TYPE_B)
    ]
    ids = manager.create_objects(specs)
    
    small_obj = manager.get_object(ids[0])
    large_obj = manager.get_object(ids[1])
    
    small_zone = manager.get_target_zone(small_obj, SortingCriteria.BY_SIZE)
    large_zone = manager.get_target_zone(large_obj, SortingCriteria.BY_SIZE)
    
    assert small_zone is not None
    assert small_zone.id == "small_zone"
    assert large_zone is not None
    assert large_zone.id == "large_zone"


def test_get_target_zone_by_type():
    """Test get_target_zone with type sorting criteria."""
    source = SourceArea(
        bounds=BoundingBox(min=Position2D(0, 0), max=Position2D(10, 10)),
        capacity=20
    )
    zones = [
        TargetZone(
            id="type_a_zone",
            bounds=BoundingBox(min=Position2D(20, 0), max=Position2D(30, 10)),
            capacity=5,
            criteria_value="type_a"
        ),
        TargetZone(
            id="type_b_zone",
            bounds=BoundingBox(min=Position2D(20, 15), max=Position2D(30, 25)),
            capacity=5,
            criteria_value="type_b"
        )
    ]
    manager = ObjectManager(source, zones)
    
    specs = [
        ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A),
        ObjectSpec(Color.BLUE, Size.LARGE, ObjectType.TYPE_B)
    ]
    ids = manager.create_objects(specs)
    
    type_a_obj = manager.get_object(ids[0])
    type_b_obj = manager.get_object(ids[1])
    
    type_a_zone = manager.get_target_zone(type_a_obj, SortingCriteria.BY_TYPE)
    type_b_zone = manager.get_target_zone(type_b_obj, SortingCriteria.BY_TYPE)
    
    assert type_a_zone is not None
    assert type_a_zone.id == "type_a_zone"
    assert type_b_zone is not None
    assert type_b_zone.id == "type_b_zone"


def test_get_target_zone_primary_criteria_priority():
    """Test that get_target_zone uses only primary criteria, ignoring other properties."""
    source = SourceArea(
        bounds=BoundingBox(min=Position2D(0, 0), max=Position2D(10, 10)),
        capacity=20
    )
    # Create zones for both color and size
    zones = [
        TargetZone(
            id="red_zone",
            bounds=BoundingBox(min=Position2D(20, 0), max=Position2D(30, 10)),
            capacity=5,
            criteria_value="red"
        ),
        TargetZone(
            id="small_zone",
            bounds=BoundingBox(min=Position2D(40, 0), max=Position2D(50, 10)),
            capacity=5,
            criteria_value="small"
        )
    ]
    manager = ObjectManager(source, zones)
    
    # Create object that is both RED and SMALL
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
    ids = manager.create_objects(specs)
    obj = manager.get_object(ids[0])
    
    # When sorting by color, should go to red_zone (not small_zone)
    zone_by_color = manager.get_target_zone(obj, SortingCriteria.BY_COLOR)
    assert zone_by_color is not None
    assert zone_by_color.id == "red_zone"
    
    # When sorting by size, should go to small_zone (not red_zone)
    zone_by_size = manager.get_target_zone(obj, SortingCriteria.BY_SIZE)
    assert zone_by_size is not None
    assert zone_by_size.id == "small_zone"


def test_get_target_zone_no_matching_zone():
    """Test get_target_zone returns None when no matching zone exists."""
    source = SourceArea(
        bounds=BoundingBox(min=Position2D(0, 0), max=Position2D(10, 10)),
        capacity=20
    )
    zones = [
        TargetZone(
            id="red_zone",
            bounds=BoundingBox(min=Position2D(20, 0), max=Position2D(30, 10)),
            capacity=5,
            criteria_value="red"
        )
    ]
    manager = ObjectManager(source, zones)
    
    # Create a GREEN object, but only red zone exists
    specs = [ObjectSpec(Color.GREEN, Size.SMALL, ObjectType.TYPE_A)]
    ids = manager.create_objects(specs)
    obj = manager.get_object(ids[0])
    
    zone = manager.get_target_zone(obj, SortingCriteria.BY_COLOR)
    assert zone is None


def test_is_zone_full_empty_zone(target_zones):
    """Test is_zone_full returns False for empty zone."""
    object_manager = ObjectManager(
        SourceArea(
            bounds=BoundingBox(min=Position2D(0, 0), max=Position2D(10, 10)),
            capacity=20
        ),
        target_zones
    )
    
    assert object_manager.is_zone_full(target_zones[0]) is False


def test_is_zone_full_partially_filled(object_manager, target_zones):
    """Test is_zone_full returns False for partially filled zone."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A) for _ in range(3)]
    ids = object_manager.create_objects(specs)
    
    zone = target_zones[0]  # capacity = 5
    
    # Fill zone partially
    for i in range(3):
        object_manager.remove_from_source(ids[i])
        object_manager.place_in_zone(ids[i], zone)
    
    assert object_manager.is_zone_full(zone) is False


def test_is_zone_full_at_capacity(object_manager, target_zones):
    """Test is_zone_full returns True when zone is at capacity."""
    specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A) for _ in range(5)]
    ids = object_manager.create_objects(specs)
    
    zone = target_zones[0]  # capacity = 5
    
    # Fill zone to capacity
    for i in range(5):
        object_manager.remove_from_source(ids[i])
        object_manager.place_in_zone(ids[i], zone)
    
    assert object_manager.is_zone_full(zone) is True
