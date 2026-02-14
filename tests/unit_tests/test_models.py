"""Unit tests for core data models."""

import pytest
from robotic_sorting.models import (
    Position2D, BoundingBox, SourceArea, TargetZone, Object,
    Color, Size, ObjectType, ObjectState
)


class TestPosition2D:
    """Tests for Position2D class."""

    def test_distance_calculation(self):
        """Test distance calculation between two positions."""
        p1 = Position2D(0.0, 0.0)
        p2 = Position2D(3.0, 4.0)
        assert p1.distance_to(p2) == 5.0

    def test_distance_to_self(self):
        """Test distance to same position is zero."""
        p = Position2D(1.0, 2.0)
        assert p.distance_to(p) == 0.0


class TestBoundingBox:
    """Tests for BoundingBox class."""

    def test_contains_point_inside(self):
        """Test that a point inside the box is detected."""
        box = BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0))
        assert box.contains(Position2D(5.0, 5.0))

    def test_contains_point_outside(self):
        """Test that a point outside the box is detected."""
        box = BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0))
        assert not box.contains(Position2D(15.0, 5.0))

    def test_contains_point_on_boundary(self):
        """Test that a point on the boundary is considered inside."""
        box = BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0))
        assert box.contains(Position2D(0.0, 0.0))
        assert box.contains(Position2D(10.0, 10.0))

    def test_intersects_overlapping_boxes(self):
        """Test that overlapping boxes are detected."""
        box1 = BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0))
        box2 = BoundingBox(Position2D(5.0, 5.0), Position2D(15.0, 15.0))
        assert box1.intersects(box2)
        assert box2.intersects(box1)

    def test_intersects_non_overlapping_boxes(self):
        """Test that non-overlapping boxes are detected."""
        box1 = BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0))
        box2 = BoundingBox(Position2D(20.0, 20.0), Position2D(30.0, 30.0))
        assert not box1.intersects(box2)

    def test_intersects_touching_boxes(self):
        """Test that boxes touching at edges are considered intersecting."""
        box1 = BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0))
        box2 = BoundingBox(Position2D(10.0, 0.0), Position2D(20.0, 10.0))
        # Touching at edge - considered intersecting for collision detection
        assert box1.intersects(box2)


class TestTargetZone:
    """Tests for TargetZone class."""

    def test_is_full_empty_zone(self):
        """Test that an empty zone is not full."""
        zone = TargetZone("zone1", BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0)),
                         capacity=5, criteria_value="red", current_count=0)
        assert not zone.is_full()

    def test_is_full_at_capacity(self):
        """Test that a zone at capacity is full."""
        zone = TargetZone("zone1", BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0)),
                         capacity=5, criteria_value="red", current_count=5)
        assert zone.is_full()

    def test_add_object_success(self):
        """Test adding an object to a non-full zone."""
        zone = TargetZone("zone1", BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0)),
                         capacity=5, criteria_value="red", current_count=0)
        assert zone.add_object()
        assert zone.current_count == 1

    def test_add_object_to_full_zone(self):
        """Test that adding to a full zone fails."""
        zone = TargetZone("zone1", BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0)),
                         capacity=5, criteria_value="red", current_count=5)
        assert not zone.add_object()
        assert zone.current_count == 5

    def test_remove_object_success(self):
        """Test removing an object from a zone."""
        zone = TargetZone("zone1", BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0)),
                         capacity=5, criteria_value="red", current_count=3)
        assert zone.remove_object()
        assert zone.current_count == 2

    def test_remove_object_from_empty_zone(self):
        """Test that removing from an empty zone fails."""
        zone = TargetZone("zone1", BoundingBox(Position2D(0.0, 0.0), Position2D(10.0, 10.0)),
                         capacity=5, criteria_value="red", current_count=0)
        assert not zone.remove_object()
        assert zone.current_count == 0


class TestObject:
    """Tests for Object class."""

    def test_object_creation(self):
        """Test creating an object with all properties."""
        obj = Object(
            id="obj1",
            color=Color.RED,
            size=Size.MEDIUM,
            type=ObjectType.TYPE_A,
            position=Position2D(5.0, 5.0)
        )
        assert obj.id == "obj1"
        assert obj.color == Color.RED
        assert obj.size == Size.MEDIUM
        assert obj.type == ObjectType.TYPE_A
        assert obj.state == ObjectState.IN_SOURCE

    def test_object_default_state(self):
        """Test that objects default to IN_SOURCE state."""
        obj = Object(
            id="obj1",
            color=Color.BLUE,
            size=Size.SMALL,
            type=ObjectType.TYPE_B,
            position=Position2D(0.0, 0.0)
        )
        assert obj.state == ObjectState.IN_SOURCE
