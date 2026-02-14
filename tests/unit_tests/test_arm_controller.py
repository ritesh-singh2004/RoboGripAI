"""Unit tests for RoboticArmController."""

import pytest
from robotic_sorting.arm_controller import RoboticArmController, ArmState
from robotic_sorting.models import Object, Position2D, Color, Size, ObjectType, ObjectState


class TestRoboticArmController:
    """Test suite for RoboticArmController class."""
    
    def test_initial_state_is_idle(self):
        """Test that arm starts in IDLE state."""
        arm = RoboticArmController()
        assert arm.get_state() == ArmState.IDLE
    
    def test_initial_position_is_origin(self):
        """Test that arm starts at origin position."""
        arm = RoboticArmController()
        assert arm.position.x == 0.0
        assert arm.position.y == 0.0
    
    def test_initial_held_object_is_none(self):
        """Test that arm starts with no held object."""
        arm = RoboticArmController()
        assert arm.held_object is None
    
    def test_initial_target_position_is_none(self):
        """Test that arm starts with no target position."""
        arm = RoboticArmController()
        assert arm.target_position is None
    
    def test_initial_retry_count_is_zero(self):
        """Test that arm starts with zero retry count."""
        arm = RoboticArmController()
        assert arm.retry_count == 0
    
    def test_reset_returns_to_idle_state(self):
        """Test that reset returns arm to IDLE state."""
        arm = RoboticArmController()
        # Manually change state to simulate operation
        arm._state = ArmState.MOVING
        arm.reset()
        assert arm.get_state() == ArmState.IDLE
    
    def test_reset_clears_position(self):
        """Test that reset returns arm to origin position."""
        arm = RoboticArmController()
        arm._position = Position2D(10.0, 20.0)
        arm.reset()
        assert arm.position.x == 0.0
        assert arm.position.y == 0.0
    
    def test_reset_clears_held_object(self):
        """Test that reset clears any held object."""
        arm = RoboticArmController()
        obj = Object(
            id="obj1",
            color=Color.RED,
            size=Size.SMALL,
            type=ObjectType.TYPE_A,
            position=Position2D(5.0, 5.0)
        )
        arm._held_object = obj
        arm.reset()
        assert arm.held_object is None
    
    def test_reset_clears_target_position(self):
        """Test that reset clears target position."""
        arm = RoboticArmController()
        arm._target_position = Position2D(15.0, 25.0)
        arm.reset()
        assert arm.target_position is None
    
    def test_reset_clears_retry_count(self):
        """Test that reset clears retry count."""
        arm = RoboticArmController()
        arm._retry_count = 3
        arm.reset()
        assert arm.retry_count == 0
    
    def test_arm_state_enum_values(self):
        """Test that ArmState enum has all required values."""
        assert ArmState.IDLE.value == "idle"
        assert ArmState.MOVING.value == "moving"
        assert ArmState.PICKING.value == "picking"
        assert ArmState.HOLDING.value == "holding"
        assert ArmState.PLACING.value == "placing"


class TestPickOperation:
    """Test suite for pick_object operation."""
    
    def test_successful_pick_on_first_attempt(self):
        """Test successful pick operation without retries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, TargetZone, OperationType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zones = []
        obj_mgr = ObjectManager(source, zones)
        
        # Create an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Pick the object
        arm = RoboticArmController()
        result = arm.pick_object(obj_id, obj_mgr, max_retries=3)
        
        # Verify success
        assert result.success is True
        assert result.operation_type == OperationType.PICK
        assert result.object_id == obj_id
        assert result.error is None
        assert result.retry_count == 0
        assert arm.get_state() == ArmState.HOLDING
        assert arm.held_object is not None
        assert arm.held_object.id == obj_id
    
    def test_pick_with_retry_success(self):
        """Test pick operation that succeeds after retries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, OperationType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Create a callback that fails twice then succeeds
        attempt_count = [0]
        def failure_callback():
            attempt_count[0] += 1
            return attempt_count[0] <= 2  # Fail first 2 attempts
        
        # Pick the object
        arm = RoboticArmController()
        result = arm.pick_object(obj_id, obj_mgr, max_retries=3, 
                                failure_callback=failure_callback)
        
        # Verify success after retries
        assert result.success is True
        assert result.retry_count == 2
        assert arm.get_state() == ArmState.HOLDING
        assert arm.held_object.id == obj_id
    
    def test_pick_fails_after_max_retries(self):
        """Test pick operation that fails after exhausting retries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, ErrorType, OperationType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Create a callback that always fails
        def always_fail():
            return True
        
        # Pick the object
        arm = RoboticArmController()
        result = arm.pick_object(obj_id, obj_mgr, max_retries=3, 
                                failure_callback=always_fail)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.MAX_RETRIES_EXCEEDED
        assert result.retry_count == 4  # Initial attempt + 3 retries
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
    
    def test_pick_from_non_idle_state_fails(self):
        """Test that pick operation fails if arm is not in IDLE state."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, ErrorType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Set arm to non-idle state
        arm = RoboticArmController()
        arm._state = ArmState.MOVING
        
        # Attempt to pick
        result = arm.pick_object(obj_id, obj_mgr)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.INVALID_STATE
        assert result.retry_count == 0
    
    def test_pick_nonexistent_object_fails(self):
        """Test that picking a non-existent object fails."""
        from robotic_sorting.object_manager import ObjectManager
        from robotic_sorting.models import BoundingBox, SourceArea, ErrorType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Attempt to pick non-existent object
        arm = RoboticArmController()
        result = arm.pick_object("nonexistent", obj_mgr)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.OBJECT_NOT_FOUND
        assert result.retry_count == 0
    
    def test_pick_object_not_in_source_fails(self):
        """Test that picking an object not in source area fails."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, ErrorType, ObjectState
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Change object state to HELD
        obj = obj_mgr.get_object(obj_id)
        obj.state = ObjectState.HELD
        
        # Attempt to pick
        arm = RoboticArmController()
        result = arm.pick_object(obj_id, obj_mgr)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.INVALID_STATE
        assert result.retry_count == 0
    
    def test_pick_removes_object_from_source(self):
        """Test that successful pick removes object from source area."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Verify object is in source
        assert len(obj_mgr.get_available_objects()) == 1
        
        # Pick the object
        arm = RoboticArmController()
        result = arm.pick_object(obj_id, obj_mgr)
        
        # Verify object removed from source
        assert result.success is True
        assert len(obj_mgr.get_available_objects()) == 0
    
    def test_pick_updates_arm_position(self):
        """Test that pick operation moves arm to object position."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(5.0, 7.0))]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        # Pick the object
        arm = RoboticArmController()
        result = arm.pick_object(obj_id, obj_mgr)
        
        # Verify arm moved to object position
        assert result.success is True
        assert arm.position.x == 5.0
        assert arm.position.y == 7.0


class TestPlaceOperation:
    """Test suite for place_object operation."""
    
    def test_successful_place_on_first_attempt(self):
        """Test successful place operation without retries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import (
            BoundingBox, SourceArea, TargetZone, OperationType
        )
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        pick_result = arm.pick_object(obj_id, obj_mgr)
        assert pick_result.success is True
        
        # Place the object
        place_result = arm.place_object(zone, obj_mgr, max_retries=3)
        
        # Verify success
        assert place_result.success is True
        assert place_result.operation_type == OperationType.PLACE
        assert place_result.object_id == obj_id
        assert place_result.error is None
        assert place_result.retry_count == 0
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
    
    def test_place_with_retry_success(self):
        """Test place operation that succeeds after retries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, TargetZone
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        arm.pick_object(obj_id, obj_mgr)
        
        # Create a callback that fails twice then succeeds
        attempt_count = [0]
        def failure_callback():
            attempt_count[0] += 1
            return attempt_count[0] <= 2  # Fail first 2 attempts
        
        # Place the object
        place_result = arm.place_object(zone, obj_mgr, max_retries=3,
                                       failure_callback=failure_callback)
        
        # Verify success after retries
        assert place_result.success is True
        assert place_result.retry_count == 2
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
    
    def test_place_fails_after_max_retries(self):
        """Test place operation that fails after exhausting retries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import (
            BoundingBox, SourceArea, TargetZone, ErrorType, ObjectState
        )
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        arm.pick_object(obj_id, obj_mgr)
        
        # Create a callback that always fails
        def always_fail():
            return True
        
        # Place the object
        place_result = arm.place_object(zone, obj_mgr, max_retries=3,
                                       failure_callback=always_fail)
        
        # Verify failure and return to source
        assert place_result.success is False
        assert place_result.error == ErrorType.MAX_RETRIES_EXCEEDED
        assert place_result.retry_count == 4  # Initial attempt + 3 retries
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
        
        # Verify object returned to source
        obj = obj_mgr.get_object(obj_id)
        assert obj.state == ObjectState.IN_SOURCE
    
    def test_place_from_non_holding_state_fails(self):
        """Test that place operation fails if arm is not in HOLDING state."""
        from robotic_sorting.object_manager import ObjectManager
        from robotic_sorting.models import BoundingBox, SourceArea, TargetZone, ErrorType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Arm in IDLE state (not holding)
        arm = RoboticArmController()
        
        # Attempt to place
        result = arm.place_object(zone, obj_mgr)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.INVALID_STATE
        assert result.retry_count == 0
    
    def test_place_in_full_zone_fails(self):
        """Test that placing in a full zone fails and returns object to source."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import (
            BoundingBox, SourceArea, TargetZone, ErrorType, ObjectState
        )
        
        # Setup with zone capacity of 0 (full)
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=0,  # Zone is full
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        arm.pick_object(obj_id, obj_mgr)
        
        # Attempt to place in full zone
        result = arm.place_object(zone, obj_mgr)
        
        # Verify failure
        assert result.success is False
        assert result.error == ErrorType.ZONE_FULL
        assert result.retry_count == 0
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
        
        # Verify object returned to source
        obj = obj_mgr.get_object(obj_id)
        assert obj.state == ObjectState.IN_SOURCE
    
    def test_place_updates_arm_position(self):
        """Test that place operation moves arm to zone position."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, TargetZone
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        arm.pick_object(obj_id, obj_mgr)
        
        # Place the object
        result = arm.place_object(zone, obj_mgr)
        
        # Verify arm moved to zone center
        assert result.success is True
        assert arm.position.x == 25.0  # Center of zone (20+30)/2
        assert arm.position.y == 25.0  # Center of zone (20+30)/2
    
    def test_place_verifies_object_in_zone_boundaries(self):
        """Test that place operation verifies object is within zone boundaries."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, TargetZone, ObjectState
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        arm.pick_object(obj_id, obj_mgr)
        
        # Place the object
        result = arm.place_object(zone, obj_mgr)
        
        # Verify object is in zone and within boundaries
        assert result.success is True
        obj = obj_mgr.get_object(obj_id)
        assert obj.state == ObjectState.IN_ZONE
        assert zone.bounds.contains(obj.position)
    
    def test_place_increments_zone_count(self):
        """Test that successful place increments zone object count."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, TargetZone
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(20, 20), Position2D(30, 30)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create and pick an object
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A)]
        obj_ids = obj_mgr.create_objects(specs)
        obj_id = obj_ids[0]
        
        arm = RoboticArmController()
        arm.pick_object(obj_id, obj_mgr)
        
        # Verify zone is empty
        assert zone.current_count == 0
        
        # Place the object
        result = arm.place_object(zone, obj_mgr)
        
        # Verify zone count incremented
        assert result.success is True
        assert zone.current_count == 1


class TestCollisionDetection:
    """Test suite for collision detection functionality."""
    
    def test_check_collision_detects_object_in_path(self):
        """Test that check_collision detects an object in the movement path."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create an object at position (5, 5)
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(5.0, 5.0))]
        obj_mgr.create_objects(specs)
        
        # Check collision on path that goes through the object
        arm = RoboticArmController()
        start = Position2D(0.0, 0.0)
        end = Position2D(10.0, 10.0)
        
        # Should detect collision
        assert arm.check_collision(start, end, obj_mgr) is True
    
    def test_check_collision_no_collision_on_clear_path(self):
        """Test that check_collision returns False when path is clear."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create an object far from the path
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(50.0, 50.0))]
        obj_mgr.create_objects(specs)
        
        # Check collision on path that doesn't go near the object
        arm = RoboticArmController()
        start = Position2D(0.0, 0.0)
        end = Position2D(10.0, 10.0)
        
        # Should not detect collision
        assert arm.check_collision(start, end, obj_mgr) is False
    
    def test_check_collision_ignores_held_object(self):
        """Test that check_collision ignores the object being held by the arm."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, ObjectState
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create an object and mark it as held
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(5.0, 5.0))]
        obj_ids = obj_mgr.create_objects(specs)
        obj = obj_mgr.get_object(obj_ids[0])
        obj.state = ObjectState.HELD
        
        # Check collision on path that goes through the held object
        arm = RoboticArmController()
        start = Position2D(0.0, 0.0)
        end = Position2D(10.0, 10.0)
        
        # Should not detect collision (held object is ignored)
        assert arm.check_collision(start, end, obj_mgr) is False
    
    def test_check_collision_with_multiple_objects(self):
        """Test collision detection with multiple objects in the scene."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create multiple objects
        specs = [
            ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(50.0, 50.0)),
            ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, Position2D(5.0, 5.0)),
            ObjectSpec(Color.GREEN, Size.LARGE, ObjectType.TYPE_C, Position2D(60.0, 60.0))
        ]
        obj_mgr.create_objects(specs)
        
        # Check collision on path that goes through one object
        arm = RoboticArmController()
        start = Position2D(0.0, 0.0)
        end = Position2D(10.0, 10.0)
        
        # Should detect collision with the object at (5, 5)
        assert arm.check_collision(start, end, obj_mgr) is True
    
    def test_move_to_succeeds_on_clear_path(self):
        """Test that move_to succeeds when path is clear."""
        from robotic_sorting.object_manager import ObjectManager
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup with no objects
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Move to target
        arm = RoboticArmController()
        target = Position2D(15.0, 20.0)
        result = arm.move_to(target, obj_mgr)
        
        # Verify success
        assert result.success is True
        assert result.error is None
        assert arm.position.x == 15.0
        assert arm.position.y == 20.0
    
    def test_move_to_stops_on_collision(self):
        """Test that move_to stops when collision is detected and no alternative path exists."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, ErrorType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(15, 15)),
            capacity=50
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create a truly impassable wall of obstacles
        # Cover a large area that blocks all reasonable paths
        specs = []
        for x in range(2, 12):
            for y in range(2, 12):
                specs.append(ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(float(x), float(y))))
        obj_mgr.create_objects(specs)
        
        # Attempt to move through the dense obstacle field
        arm = RoboticArmController()
        initial_position = arm.position
        target = Position2D(10.0, 10.0)
        result = arm.move_to(target, obj_mgr)
        
        # Verify collision detected and arm stopped (no alternative path exists)
        assert result.success is False
        assert result.error == ErrorType.COLLISION
        assert arm.position.x == initial_position.x
        assert arm.position.y == initial_position.y
        assert arm.get_state() == ArmState.IDLE
    
    def test_move_to_returns_to_idle_on_collision(self):
        """Test that move_to returns arm to IDLE state on collision when no alternative path exists."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(15, 15)),
            capacity=50
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create a truly impassable wall of obstacles
        specs = []
        for x in range(2, 12):
            for y in range(2, 12):
                specs.append(ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(float(x), float(y))))
        obj_mgr.create_objects(specs)
        
        # Set arm to MOVING state
        arm = RoboticArmController()
        arm._state = ArmState.MOVING
        
        # Attempt to move through the dense obstacle field
        target = Position2D(10.0, 10.0)
        result = arm.move_to(target, obj_mgr)
        
        # Verify arm returned to IDLE (no alternative path exists)
        assert result.success is False
        assert arm.get_state() == ArmState.IDLE
    
    def test_pick_detects_collision_during_movement(self):
        """Test that pick operation detects collision during movement when no alternative path exists."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea, ErrorType
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(25, 25)),
            capacity=100
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create target object at (20, 20) and truly impassable obstacle field
        specs = [
            ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(20.0, 20.0))
        ]
        # Create massive obstacle grid that blocks all paths
        for x in range(2, 18):
            for y in range(2, 18):
                specs.append(ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, 
                          Position2D(float(x), float(y))))
        
        obj_ids = obj_mgr.create_objects(specs)
        target_id = obj_ids[0]
        
        # Attempt to pick object (no alternative path exists through dense obstacles)
        arm = RoboticArmController()
        result = arm.pick_object(target_id, obj_mgr)
        
        # Verify collision detected
        assert result.success is False
        assert result.error == ErrorType.COLLISION
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
    
    def test_place_detects_collision_during_movement(self):
        """Test that place operation detects collision during movement when no alternative path exists."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import (
            BoundingBox, SourceArea, TargetZone, ErrorType, ObjectState
        )
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        # Place zone far to the right
        zone = TargetZone(
            id="zone1",
            bounds=BoundingBox(Position2D(40, 0), Position2D(50, 10)),
            capacity=10,
            criteria_value="red"
        )
        obj_mgr = ObjectManager(source, [zone])
        
        # Create object at (5, 5) and dense obstacle field blocking all paths to zone
        specs = [
            ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, Position2D(5.0, 5.0))
        ]
        # Create dense obstacle grid that blocks all paths to zone
        for x in range(15, 25):
            for y in range(2, 8):
                specs.append(ObjectSpec(Color.BLUE, Size.MEDIUM, ObjectType.TYPE_B, 
                          Position2D(float(x), float(y))))
        
        obj_ids = obj_mgr.create_objects(specs)
        target_id = obj_ids[0]
        
        # Pick the object first
        arm = RoboticArmController()
        pick_result = arm.pick_object(target_id, obj_mgr)
        assert pick_result.success is True
        
        # Attempt to place (no alternative path exists through dense obstacles)
        place_result = arm.place_object(zone, obj_mgr)
        
        # Verify collision detected and object returned to source
        assert place_result.success is False
        assert place_result.error == ErrorType.COLLISION
        # Verify object was returned to source
        obj = obj_mgr.get_object(target_id)
        assert obj.state == ObjectState.IN_SOURCE
        
        # Verify collision detected and object returned to source
        assert place_result.success is False
        assert place_result.error == ErrorType.COLLISION
        assert arm.get_state() == ArmState.IDLE
        assert arm.held_object is None
        
        # Verify object returned to source
        obj = obj_mgr.get_object(target_id)
        assert obj.state == ObjectState.IN_SOURCE
    
    def test_collision_with_buffer_zone(self):
        """Test that collision detection includes buffer zone around path."""
        from robotic_sorting.object_manager import ObjectManager, ObjectSpec
        from robotic_sorting.models import BoundingBox, SourceArea
        
        # Setup
        source = SourceArea(
            bounds=BoundingBox(Position2D(0, 0), Position2D(10, 10)),
            capacity=10
        )
        obj_mgr = ObjectManager(source, [])
        
        # Create an object just outside the direct path but within buffer (0.5 units)
        # Path from (0,0) to (10,0) has buffer extending to y=0.5
        specs = [ObjectSpec(Color.RED, Size.SMALL, ObjectType.TYPE_A, 
                          Position2D(5.0, 0.3))]
        obj_mgr.create_objects(specs)
        
        # Check collision on horizontal path
        arm = RoboticArmController()
        start = Position2D(0.0, 0.0)
        end = Position2D(10.0, 0.0)
        
        # Should detect collision due to buffer zone
        assert arm.check_collision(start, end, obj_mgr) is True
