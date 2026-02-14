"""Object Manager component for tracking objects and their locations."""

from typing import Dict, List, Optional
from robotic_sorting.models import (
    Object, ObjectState, SourceArea, TargetZone, Position2D,
    Color, Size, ObjectType, SortingCriteria
)


class ObjectSpec:
    """Specification for creating an object."""
    def __init__(self, color: Color, size: Size, obj_type: ObjectType, 
                 initial_position: Optional[Position2D] = None):
        self.color = color
        self.size = size
        self.type = obj_type
        self.initial_position = initial_position


class ObjectManager:
    """Manages all objects in the simulation and their locations."""
    
    def __init__(self, source_area: SourceArea, target_zones: List[TargetZone]):
        """Initialize the object manager.
        
        Args:
            source_area: The source area where objects start
            target_zones: List of target zones for sorted objects
        """
        self.objects: Dict[str, Object] = {}
        self.source_area = source_area
        self.target_zones = target_zones
        self._next_id = 0
    
    def create_objects(self, specs: List[ObjectSpec]) -> List[str]:
        """Create objects from specifications.
        
        Args:
            specs: List of object specifications
            
        Returns:
            List of created object IDs
        """
        created_ids = []
        
        for spec in specs:
            # Generate unique ID
            obj_id = f"obj_{self._next_id}"
            self._next_id += 1
            
            # Use provided position or default to source area center
            if spec.initial_position:
                position = spec.initial_position
            else:
                # Default to center of source area
                position = Position2D(
                    x=(self.source_area.bounds.min.x + self.source_area.bounds.max.x) / 2,
                    y=(self.source_area.bounds.min.y + self.source_area.bounds.max.y) / 2
                )
            
            # Create object
            obj = Object(
                id=obj_id,
                color=spec.color,
                size=spec.size,
                type=spec.type,
                position=position,
                state=ObjectState.IN_SOURCE
            )
            
            self.objects[obj_id] = obj
            created_ids.append(obj_id)
        
        return created_ids

    def get_object(self, object_id: str) -> Optional[Object]:
        """Get an object by ID.
        
        Args:
            object_id: The object ID to retrieve
            
        Returns:
            The object if found, None otherwise
        """
        return self.objects.get(object_id)
    
    def get_available_objects(self) -> List[Object]:
        """Get all objects currently in the source area.
        
        Returns:
            List of objects in the source area
        """
        return [obj for obj in self.objects.values() 
                if obj.state == ObjectState.IN_SOURCE]
    
    def remove_from_source(self, object_id: str) -> bool:
        """Remove an object from the source area (when picked).
        
        Args:
            object_id: The object ID to remove
            
        Returns:
            True if successful, False if object not found or not in source
        """
        obj = self.objects.get(object_id)
        if obj is None:
            return False
        
        if obj.state != ObjectState.IN_SOURCE:
            return False
        
        obj.state = ObjectState.HELD
        return True
    
    def place_in_zone(self, object_id: str, zone: TargetZone) -> bool:
        """Place an object in a target zone.
        
        Args:
            object_id: The object ID to place
            zone: The target zone to place the object in
            
        Returns:
            True if successful, False if zone is full or object not held
        """
        obj = self.objects.get(object_id)
        if obj is None:
            return False
        
        if obj.state != ObjectState.HELD:
            return False
        
        if zone.is_full():
            return False
        
        # Add object to zone
        if not zone.add_object():
            return False
        
        # Update object state and position to zone center
        obj.state = ObjectState.IN_ZONE
        obj.position = Position2D(
            x=(zone.bounds.min.x + zone.bounds.max.x) / 2,
            y=(zone.bounds.min.y + zone.bounds.max.y) / 2
        )
        
        return True
    
    def return_to_source(self, object_id: str) -> bool:
        """Return an object to the source area (after failed operation).
        
        Args:
            object_id: The object ID to return
            
        Returns:
            True if successful, False if object not found or not held
        """
        obj = self.objects.get(object_id)
        if obj is None:
            return False
        
        if obj.state != ObjectState.HELD:
            return False
        
        # Return to source area center
        obj.state = ObjectState.IN_SOURCE
        obj.position = Position2D(
            x=(self.source_area.bounds.min.x + self.source_area.bounds.max.x) / 2,
            y=(self.source_area.bounds.min.y + self.source_area.bounds.max.y) / 2
        )
        
        return True

    def get_target_zone(self, obj: Object, criteria: SortingCriteria) -> Optional[TargetZone]:
        """Determine the appropriate target zone for an object based on sorting criteria.
        
        This method implements the primary criteria priority: when an object has properties
        that could match multiple zones, only the primary sorting criteria is used to
        determine placement.
        
        Args:
            obj: The object to find a target zone for
            criteria: The sorting criteria to use (color, size, or type)
            
        Returns:
            The target zone matching the object's property for the given criteria,
            or None if no matching zone is found
        """
        # Determine the property value to match based on criteria
        if criteria == SortingCriteria.BY_COLOR:
            criteria_value = obj.color.value
        elif criteria == SortingCriteria.BY_SIZE:
            criteria_value = obj.size.value
        elif criteria == SortingCriteria.BY_TYPE:
            criteria_value = obj.type.value
        else:
            return None
        
        # Find the first zone that matches the criteria value
        for zone in self.target_zones:
            if zone.criteria_value == criteria_value:
                return zone
        
        return None
    
    def is_zone_full(self, zone: TargetZone) -> bool:
        """Check if a target zone is at capacity.
        
        Args:
            zone: The target zone to check
            
        Returns:
            True if the zone is full, False otherwise
        """
        return zone.is_full()
