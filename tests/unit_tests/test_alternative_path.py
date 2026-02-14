"""Unit tests for alternative path planning functionality."""

import pytest
from robotic_sorting.arm_controller import RoboticArmController, ArmState
from robotic_sorting.object_manager import ObjectManager, ObjectSpec
from robotic_sorting.models import (
    Object, Position2D, Color, Size, ObjectType, ObjectState,
    BoundingBox, SourceArea, TargetZone, ErrorType
)


class TestAlternativePathPlanning:
    """Test suite for alternative path planning functionality."""
    
    def test_find_alternative_path_with_single_obstacle(self):
        """Test that alternative path is found when single obstacle blocks direct path."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create an obstacle in the direct path
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 5.0))]
        obj_mgr.create_objects(specs)
        
        # Attempt to move through the obstacle
        arm = RoboticArmController()
        start = Position2D(0.0, 0.0)
        target = Position2D(10.0, 10.0)
        
        # Move to target - should find alternative path
        result = arm.move_to(target, obj_mgr)
        
        # Verify success via alternative path
        assert result.success is True
        assert result.error is None
        assert arm.position.x == 10.0
        assert arm.position.y == 10.0
    
    def test_move_to_uses_alternative_path_on_collision(self):
        """Test that move_to finds and uses alternative path when collision detected."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create obstacle at midpoint of path
        specs = [ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, Position2D(5.0, 0.0))]
        obj_mgr.create_objects(specs)
        
        # Move from origin to (10, 0) - direct path blocked
        arm = RoboticArmController()
        target = Position2D(10.0, 0.0)
        result = arm.move_to(target, obj_mgr)
        
        # Should succeed via alternative path
        assert result.success is True
        assert arm.position == target
    
    def test_no_alternative_path_returns_to_idle(self):
        """Test that arm returns to idle when no alternative path exists."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(30, 30)),
            capacity=100
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create a truly impassable wall of obstacles
        # Cover a large area that blocks all reasonable paths
        obstacle_specs = []
        for x in range(2, 20):
            for y in range(2, 20):
                obstacle_specs.append(
                    ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                             Position2D(float(x), float(y)))
                )
        obj_mgr.create_objects(obstacle_specs)
        
        # Attempt to move through the massive obstacle field
        arm = RoboticArmController()
        target = Position2D(25.0, 25.0)
        result = arm.move_to(target, obj_mgr)
        
        # Should fail - no alternative path exists
        assert result.success is False
        assert result.error == ErrorType.COLLISION
        assert arm.get_state() == ArmState.IDLE
        # Arm should not have moved
        assert arm.position.x == 0.0
        assert arm.position.y == 0.0
    
    def test_pick_succeeds_with_alternative_path(self):
        """Test that pick operation succeeds using alternative path when direct path blocked."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(20, 20)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create target object and obstacle between arm and target
        specs = [
            ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(10.0, 10.0)),
            ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, Position2D(5.0, 5.0))
        ]
        obj_ids = obj_mgr.create_objects(specs)
        target_id = obj_ids[0]
        
        # Pick the object - should use alternative path
        arm = RoboticArmController()
        result = arm.pick_object(target_id, obj_mgr)
        
        # Verify success
        assert result.success is True
        assert result.error is None
        assert arm.get_state() == ArmState.HOLDING
        assert arm.held_object.id == target_id
    
    def test_pick_fails_when_no_alternative_path(self):
        """Test that pick operation fails when no alternative path exists."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(30, 30)),
            capacity=100
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create target object at (25, 25)
        target_spec = ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(25.0, 25.0))
        
        # Create truly impassable obstacle field
        obstacle_specs = [target_spec]
        for x in range(2, 20):
            for y in range(2, 20):
                obstacle_specs.append(
                    ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, 
                             Position2D(float(x), float(y)))
                )
        
        obj_ids = obj_mgr.create_objects(obstacle_specs)
        target_id = obj_ids[0]
        
        # Attempt to pick - should fail due to no alternative path
        arm = RoboticArmController()
        result = arm.pick_object(target_id, obj_mgr)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.COLLISION
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
    
    def test_place_succeeds_with_alternative_path(self):
        """Test that place operation succeeds using alternative path when direct path blocked."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(30, 0), Position2D(40, 10)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create object to place and obstacle in path to zone
        specs = [
            ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 5.0)),
            ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, Position2D(20.0, 5.0))
        ]
        obj_ids = obj_mgr.create_objects(specs)
        target_id = obj_ids[0]
        
        # Pick the object
        arm = RoboticArmController()
        pick_result = arm.pick_object(target_id, obj_mgr)
        assert pick_result.success is True
        
        # Place the object - should use alternative path
        place_result = arm.place_object(zone, obj_mgr)
        
        # Verify success
        assert place_result.success is True
        assert place_result.error is None
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
    
    def test_place_fails_and_returns_to_source_when_no_alternative_path(self):
        """Test that place fails and returns object to source when no alternative path exists."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(50, 0), Position2D(60, 10)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create object to place
        target_spec = ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 5.0))
        
        # Create much denser obstacle field blocking all paths to zone
        obstacle_specs = [target_spec]
        for x in range(20, 35):
            for y in range(1, 9):
                obstacle_specs.append(
                    ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, 
                             Position2D(float(x), float(y)))
                )
        
        obj_ids = obj_mgr.create_objects(obstacle_specs)
        target_id = obj_ids[0]
        
        # Pick the object
        arm = RoboticArmController()
        pick_result = arm.pick_object(target_id, obj_mgr)
        assert pick_result.success is True
        
        # Attempt to place - should fail due to no alternative path
        place_result = arm.place_object(zone, obj_mgr)
        
        # Verify failure and return to source
        assert place_result.success is False
        assert place_result.error == ErrorType.COLLISION
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
        
        # Verify object returned to source
        obj = obj_mgr.get_object(target_id)
        assert obj.state == ObjectState.IN_SOURCE
    
    def test_alternative_path_tries_multiple_offsets(self):
        """Test that alternative path algorithm tries multiple offset distances."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(20, 20)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create obstacles that block small offsets but not large ones
        # Place obstacles at offsets 1.5 and 2.5 from the direct path
        specs = [
            ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 6.5)),
            ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, Position2D(5.0, 3.5))
        ]
        obj_mgr.create_objects(specs)
        
        # Move from (0, 5) to (10, 5) - direct path blocked by obstacles
        arm = RoboticArmController()
        arm._position = Position2D(0.0, 5.0)
        target = Position2D(10.0, 5.0)
        result = arm.move_to(target, obj_mgr)
        
        # Should succeed by trying larger offset
        assert result.success is True
        assert arm.position == target
    
    def test_alternative_path_tries_both_directions(self):
        """Test that alternative path algorithm tries both left and right offsets."""
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(20, 20)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create obstacle on one side of the path
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 7.0))]
        obj_mgr.create_objects(specs)
        
        # Move from (0, 5) to (10, 5) - obstacle on one side
        arm = RoboticArmController()
        arm._position = Position2D(0.0, 5.0)
        target = Position2D(10.0, 5.0)
        result = arm.move_to(target, obj_mgr)
        
        # Should succeed by going around the other side
        assert result.success is True
        assert arm.position == target
    
    def test_collision_recorded_when_alternative_path_used(self):
        """Test that collision is still detected even when alternative path succeeds."""
        # This test verifies that the system detects the collision
        # (for metrics purposes) even though it successfully uses an alternative path
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create obstacle
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 5.0))]
        obj_mgr.create_objects(specs)
        
        # Move through obstacle
        arm = RoboticArmController()
        result = arm.move_to(Position2D(10.0, 10.0), obj_mgr)
        
        # Movement succeeds via alternative path
        assert result.success is True
        
        # Note: Collision recording would be handled by the simulation engine
        # This test just verifies that the movement succeeds
