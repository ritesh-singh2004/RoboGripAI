"""Robotic arm controller with state machine."""

from enum import Enum
from typing import Optional
from robotic_sorting.models import Object, Position2D


class ArmState(Enum):
    """Robotic arm state enumeration."""
    IDLE = "idle"
    MOVING = "moving"
    PICKING = "picking"
    HOLDING = "holding"
    PLACING = "placing"


class RoboticArmController:
    """
    Controls the robotic arm with state machine logic.
    
    State transitions:
    - Idle -> Moving: pick_object() or place_object() initiated
    - Moving -> Picking: arrived at object location
    - Moving -> Placing: arrived at zone location
    - Moving -> Idle: collision detected
    - Picking -> Holding: pick successful
    - Picking -> Idle: pick failed (max retries)
    - Picking -> Picking: pick failed (retry)
    - Holding -> Moving: move to zone
    - Placing -> Idle: place successful
    - Placing -> Holding: place failed (retry)
    - Placing -> Moving: place failed (return to source)
    """
    
    def __init__(self):
        """Initialize the robotic arm controller."""
        self._state: ArmState = ArmState.IDLE
        self._position: Position2D = Position2D(0.0, 0.0)
        self._held_object: Optional[Object] = None
        self._target_position: Optional[Position2D] = None
        self._retry_count: int = 0
    
    def get_state(self) -> ArmState:
        """
        Get the current state of the robotic arm.
        
        Returns:
            Current ArmState
        """
        return self._state
    
    def reset(self) -> None:
        """
        Reset the robotic arm to initial state.
        
        Clears all state, returns to idle position, and drops any held object.
        """
        self._state = ArmState.IDLE
        self._position = Position2D(0.0, 0.0)
        self._held_object = None
        self._target_position = None
        self._retry_count = 0
    
    @property
    def position(self) -> Position2D:
        """Get the current position of the arm."""
        return self._position
    
    @property
    def held_object(self) -> Optional[Object]:
        """Get the object currently held by the arm."""
        return self._held_object
    
    @property
    def target_position(self) -> Optional[Position2D]:
        """Get the target position the arm is moving to."""
        return self._target_position
    
    @property
    def retry_count(self) -> int:
        """Get the current retry count for operations."""
        return self._retry_count

    def check_collision(self, start: Position2D, end: Position2D,
                       object_manager, ignore_object_id: Optional[str] = None) -> bool:
        """
        Check if a path from start to end intersects with any objects.

        This method checks for collisions along the movement path by testing
        if the path bounding box intersects with any object's position.
        For simplicity, we treat objects as points and the path as a line segment
        with a small buffer zone.

        Args:
            start: Starting position of the path
            end: Ending position of the path
            object_manager: ObjectManager instance to check objects against
            ignore_object_id: Optional object ID to ignore (e.g., target object during pick)

        Returns:
            True if collision detected, False otherwise
        """
        from robotic_sorting.models import BoundingBox, ObjectState

        # Create a bounding box for the path with a small buffer (0.5 units)
        buffer = 0.5
        path_min_x = min(start.x, end.x) - buffer
        path_max_x = max(start.x, end.x) + buffer
        path_min_y = min(start.y, end.y) - buffer
        path_max_y = max(start.y, end.y) + buffer

        path_bbox = BoundingBox(
            min=Position2D(path_min_x, path_min_y),
            max=Position2D(path_max_x, path_max_y)
        )

        # Check all objects that are not held by the arm
        for obj in object_manager.objects.values():
            # Skip the object we're holding
            if obj.state == ObjectState.HELD:
                continue

            # Skip the object we're explicitly ignoring (e.g., target during pick)
            if ignore_object_id and obj.id == ignore_object_id:
                continue

            # Check if object position is within the path bounding box
            if path_bbox.contains(obj.position):
                # Additional check: is the object close to the line segment?
                # For simplicity, if it's in the bounding box, consider it a collision
                return True

        return False

    def _find_alternative_path(self, start: Position2D, end: Position2D,
                               object_manager, ignore_object_id: Optional[str] = None) -> Optional[list[Position2D]]:
        """
        Find alternative waypoints to avoid collision.

        This implements a simple alternative path algorithm by trying waypoints
        around the direct path. Due to the bounding box collision detection,
        it may need to use two waypoints to route around obstacles.

        Strategy:
        1. Try single waypoints offset perpendicular to the path
        2. If that fails, try two-waypoint paths that go around obstacles

        Args:
            start: Starting position
            end: Target position
            object_manager: ObjectManager instance to check for collisions
            ignore_object_id: Optional object ID to ignore during collision detection

        Returns:
            List of waypoint positions if found, None if no alternative exists
        """
        # Calculate direction vector
        dx = end.x - start.x
        dy = end.y - start.y
        
        # Calculate distance
        distance = (dx ** 2 + dy ** 2) ** 0.5
        
        # Handle zero-distance case
        if distance < 0.01:
            return None
        
        # Normalize direction vector
        norm_dx = dx / distance
        norm_dy = dy / distance
        
        # Perpendicular vector (rotate 90 degrees)
        perp_x = -norm_dy
        perp_y = norm_dx
        
        # Strategy 1: Try single waypoints offset perpendicular to the path
        base_offsets = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0]
        
        for offset in base_offsets:
            # Try both left and right offsets
            for direction in [1, -1]:
                # Try waypoints at different positions along the path
                for fraction in [0.2, 0.33, 0.5, 0.67, 0.8]:
                    waypoint = Position2D(
                        x=start.x + dx * fraction + direction * offset * perp_x,
                        y=start.y + dy * fraction + direction * offset * perp_y
                    )
                    
                    # Check if path through waypoint is collision-free
                    path1_clear = not self.check_collision(start, waypoint, object_manager, ignore_object_id)
                    path2_clear = not self.check_collision(waypoint, end, object_manager, ignore_object_id)
                    
                    if path1_clear and path2_clear:
                        return [waypoint]
        
        # Strategy 2: Try two-waypoint paths (go perpendicular, then parallel, then back)
        # This helps with bounding box collision detection
        for offset in base_offsets:
            for direction in [1, -1]:
                # Create waypoints: one near start, one near end
                waypoint1 = Position2D(
                    x=start.x + dx * 0.3 + direction * offset * perp_x,
                    y=start.y + dy * 0.3 + direction * offset * perp_y
                )
                waypoint2 = Position2D(
                    x=start.x + dx * 0.7 + direction * offset * perp_x,
                    y=start.y + dy * 0.7 + direction * offset * perp_y
                )
                
                # Check if path through both waypoints is collision-free
                path1_clear = not self.check_collision(start, waypoint1, object_manager, ignore_object_id)
                path2_clear = not self.check_collision(waypoint1, waypoint2, object_manager, ignore_object_id)
                path3_clear = not self.check_collision(waypoint2, end, object_manager, ignore_object_id)
                
                if path1_clear and path2_clear and path3_clear:
                    return [waypoint1, waypoint2]
        
        # No alternative path found
        return None

    def move_to(self, target: Position2D, object_manager, 
                ignore_object_id: Optional[str] = None) -> 'MovementResult':
        """
        Move the arm to a target position with collision detection and alternative path planning.

        This method implements movement with collision detection. If a collision
        is detected along the direct path, it attempts to find an alternative path.
        If no alternative exists, it stops and reports the collision.

        Args:
            target: Target position to move to
            object_manager: ObjectManager instance to check for collisions
            ignore_object_id: Optional object ID to ignore during collision detection

        Returns:
            MovementResult indicating success or collision
        """
        from robotic_sorting.models import ErrorType, MovementResult

        # Check for collision along the direct path
        if self.check_collision(self._position, target, object_manager, ignore_object_id):
            # Collision detected - attempt alternative path
            waypoints = self._find_alternative_path(
                self._position, target, object_manager, ignore_object_id
            )
            
            if waypoints is not None:
                # Alternative path found - move through waypoints
                for waypoint in waypoints:
                    self._position = waypoint
                # Finally move to target
                self._position = target
                self._target_position = target
                
                return MovementResult(
                    success=True,
                    error=None,
                    final_position=self._position
                )
            else:
                # No alternative path exists - unrecoverable collision
                self._state = ArmState.IDLE
                self._target_position = None

                return MovementResult(
                    success=False,
                    error=ErrorType.COLLISION,
                    final_position=self._position
                )

        # No collision - proceed with direct movement
        self._target_position = target
        self._position = target

        return MovementResult(
            success=True,
            error=None,
            final_position=self._position
        )

    def pick_object(self, object_id: str, object_manager, 
                    max_retries: int = 3, 
                    failure_callback=None) -> 'OperationResult':
        """
        Pick an object from the source area with retry logic.
        
        This method implements the pick operation with automatic retry on failure.
        It handles missed pick detection and updates arm state appropriately.
        
        Args:
            object_id: ID of the object to pick
            object_manager: ObjectManager instance to interact with objects
            max_retries: Maximum number of retry attempts (default 3)
            failure_callback: Optional callback function that returns True if pick should fail
            
        Returns:
            OperationResult with success status, error type, and retry count
        """
        from robotic_sorting.models import OperationResult, OperationType, ErrorType
        import time
        
        # Validate state - can only pick from IDLE state
        if self._state != ArmState.IDLE:
            return OperationResult(
                success=False,
                operation_type=OperationType.PICK,
                object_id=object_id,
                error=ErrorType.INVALID_STATE,
                retry_count=0,
                timestamp=time.time()
            )
        
        # Get the object
        obj = object_manager.get_object(object_id)
        if obj is None:
            return OperationResult(
                success=False,
                operation_type=OperationType.PICK,
                object_id=object_id,
                error=ErrorType.OBJECT_NOT_FOUND,
                retry_count=0,
                timestamp=time.time()
            )
        
        # Check if object is in source area
        from robotic_sorting.models import ObjectState
        if obj.state != ObjectState.IN_SOURCE:
            return OperationResult(
                success=False,
                operation_type=OperationType.PICK,
                object_id=object_id,
                error=ErrorType.INVALID_STATE,
                retry_count=0,
                timestamp=time.time()
            )
        
        # Retry loop
        self._retry_count = 0
        while self._retry_count <= max_retries:
            # Transition to MOVING state
            self._state = ArmState.MOVING
            self._target_position = obj.position
            
            # Move to object with collision detection
            movement_result = self.move_to(obj.position, object_manager, ignore_object_id=object_id)
            
            # Check for collision during movement
            if not movement_result.success:
                # Collision detected - stop and report
                return OperationResult(
                    success=False,
                    operation_type=OperationType.PICK,
                    object_id=object_id,
                    error=ErrorType.COLLISION,
                    retry_count=self._retry_count,
                    timestamp=time.time()
                )
            
            # Transition to PICKING state
            self._state = ArmState.PICKING
            
            # Determine if pick fails (using callback if provided)
            pick_failed = False
            if failure_callback is not None:
                pick_failed = failure_callback()
            
            if not pick_failed:
                # Pick successful
                if object_manager.remove_from_source(object_id):
                    self._held_object = obj
                    self._state = ArmState.HOLDING
                    self._target_position = None
                    
                    return OperationResult(
                        success=True,
                        operation_type=OperationType.PICK,
                        object_id=object_id,
                        error=None,
                        retry_count=self._retry_count,
                        timestamp=time.time()
                    )
            
            # Pick failed - missed pick
            self._retry_count += 1
            
            # If we've exhausted retries, return failure
            if self._retry_count > max_retries:
                self._state = ArmState.IDLE
                self._target_position = None
                
                return OperationResult(
                    success=False,
                    operation_type=OperationType.PICK,
                    object_id=object_id,
                    error=ErrorType.MAX_RETRIES_EXCEEDED,
                    retry_count=self._retry_count,
                    timestamp=time.time()
                )
            
            # Continue to retry (state remains PICKING for retry)
        
        # Should not reach here, but return failure just in case
        self._state = ArmState.IDLE
        self._target_position = None
        
        return OperationResult(
            success=False,
            operation_type=OperationType.PICK,
            object_id=object_id,
            error=ErrorType.MAX_RETRIES_EXCEEDED,
            retry_count=self._retry_count,
            timestamp=time.time()
        )

    def place_object(self, zone, object_manager,
                     max_retries: int = 3,
                     failure_callback=None) -> 'OperationResult':
        """
        Place the held object in a target zone with retry logic.

        This method implements the place operation with automatic retry on failure.
        It handles zone validation, placement success/failure, and returns the object
        to source on failure.

        Args:
            zone: TargetZone to place the object in
            object_manager: ObjectManager instance to interact with objects
            max_retries: Maximum number of retry attempts (default 3)
            failure_callback: Optional callback function that returns True if place should fail

        Returns:
            OperationResult with success status, error type, and retry count
        """
        from robotic_sorting.models import OperationResult, OperationType, ErrorType
        import time

        # Validate state - can only place from HOLDING state
        if self._state != ArmState.HOLDING:
            return OperationResult(
                success=False,
                operation_type=OperationType.PLACE,
                object_id=self._held_object.id if self._held_object else None,
                error=ErrorType.INVALID_STATE,
                retry_count=0,
                timestamp=time.time()
            )

        # Must have an object to place
        if self._held_object is None:
            return OperationResult(
                success=False,
                operation_type=OperationType.PLACE,
                object_id=None,
                error=ErrorType.INVALID_STATE,
                retry_count=0,
                timestamp=time.time()
            )

        object_id = self._held_object.id

        # Check if zone is full before attempting placement
        if zone.is_full():
            # Zone full - return object to source immediately
            if object_manager.return_to_source(object_id):
                self._held_object = None
                self._state = ArmState.IDLE
                self._target_position = None

                return OperationResult(
                    success=False,
                    operation_type=OperationType.PLACE,
                    object_id=object_id,
                    error=ErrorType.ZONE_FULL,
                    retry_count=0,
                    timestamp=time.time()
                )

        # Retry loop
        self._retry_count = 0
        while self._retry_count <= max_retries:
            # Transition to MOVING state
            self._state = ArmState.MOVING

            # Calculate target position (center of zone)
            from robotic_sorting.models import Position2D
            zone_center = Position2D(
                x=(zone.bounds.min.x + zone.bounds.max.x) / 2,
                y=(zone.bounds.min.y + zone.bounds.max.y) / 2
            )
            self._target_position = zone_center

            # Move to zone with collision detection
            movement_result = self.move_to(zone_center, object_manager)
            
            # Check for collision during movement
            if not movement_result.success:
                # Collision detected - stop and report
                # Return object to source on collision
                if object_manager.return_to_source(object_id):
                    self._held_object = None
                    self._state = ArmState.IDLE
                    self._target_position = None
                
                return OperationResult(
                    success=False,
                    operation_type=OperationType.PLACE,
                    object_id=object_id,
                    error=ErrorType.COLLISION,
                    retry_count=self._retry_count,
                    timestamp=time.time()
                )

            # Transition to PLACING state
            self._state = ArmState.PLACING

            # Determine if place fails (using callback if provided)
            place_failed = False
            if failure_callback is not None:
                place_failed = failure_callback()

            if not place_failed:
                # Place successful
                if object_manager.place_in_zone(object_id, zone):
                    # Verify object is within zone boundaries
                    obj = object_manager.get_object(object_id)
                    if obj and zone.bounds.contains(obj.position):
                        self._held_object = None
                        self._state = ArmState.IDLE
                        self._target_position = None

                        return OperationResult(
                            success=True,
                            operation_type=OperationType.PLACE,
                            object_id=object_id,
                            error=None,
                            retry_count=self._retry_count,
                            timestamp=time.time()
                        )

            # Place failed
            self._retry_count += 1

            # If we've exhausted retries, return object to source
            if self._retry_count > max_retries:
                # Return to source on failure
                if object_manager.return_to_source(object_id):
                    self._held_object = None
                    self._state = ArmState.IDLE
                    self._target_position = None

                    return OperationResult(
                        success=False,
                        operation_type=OperationType.PLACE,
                        object_id=object_id,
                        error=ErrorType.MAX_RETRIES_EXCEEDED,
                        retry_count=self._retry_count,
                        timestamp=time.time()
                    )

            # Continue to retry (transition back to HOLDING for retry)
            self._state = ArmState.HOLDING

        # Should not reach here, but return failure and return to source just in case
        if object_manager.return_to_source(object_id):
            self._held_object = None
        self._state = ArmState.IDLE
        self._target_position = None

        return OperationResult(
            success=False,
            operation_type=OperationType.PLACE,
            object_id=object_id,
            error=ErrorType.MAX_RETRIES_EXCEEDED,
            retry_count=self._retry_count,
            timestamp=time.time()
        )

