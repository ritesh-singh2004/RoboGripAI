"""Metrics collection and reporting for the robotic sorting system."""

from dataclasses import dataclass, field
from typing import List, Optional
from time import time

from .models import OperationType, ErrorType


@dataclass
class OperationRecord:
    """Record of a single operation for history tracking."""
    timestamp: float
    operation_type: OperationType
    success: bool
    object_id: Optional[str] = None
    error: Optional[ErrorType] = None
    retry_count: int = 0
    correct_placement: Optional[bool] = None  # Only relevant for place operations


@dataclass
class MetricsReport:
    """Comprehensive metrics report for a simulation cycle."""
    total_objects: int
    successful_operations: int
    failed_operations: int
    success_rate: float  # percentage
    accuracy: float  # percentage of correct placements
    total_time: float  # seconds
    collisions: int
    missed_picks: int
    operations_per_second: float
    cycle_complete: bool
    successful_picks: int = 0
    failed_picks: int = 0
    successful_places: int = 0
    failed_places: int = 0
    correct_placements: int = 0
    incorrect_placements: int = 0


class MetricsCollector:
    """Collects and calculates performance metrics for the simulation."""

    def __init__(self):
        """Initialize the metrics collector with zero counters."""
        self.successful_picks: int = 0
        self.failed_picks: int = 0
        self.successful_places: int = 0
        self.failed_places: int = 0
        self.collisions: int = 0
        self.missed_picks: int = 0
        self.correct_placements: int = 0
        self.incorrect_placements: int = 0
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.operation_history: List[OperationRecord] = []
        self.metrics_history: List[MetricsReport] = []

    def record_pick(self, success: bool, missed: bool = False, 
                    object_id: Optional[str] = None, retry_count: int = 0) -> None:
        """
        Record a pick operation outcome.
        
        Args:
            success: Whether the pick was successful
            missed: Whether this was a missed pick (failed to grasp)
            object_id: ID of the object being picked
            retry_count: Number of retries attempted
        """
        if success:
            self.successful_picks += 1
        else:
            self.failed_picks += 1
            if missed:
                self.missed_picks += 1
        
        # Record in operation history
        error = ErrorType.MISSED_PICK if missed and not success else None
        record = OperationRecord(
            timestamp=time(),
            operation_type=OperationType.PICK,
            success=success,
            object_id=object_id,
            error=error,
            retry_count=retry_count
        )
        self.operation_history.append(record)

    def record_place(self, success: bool, correct_zone: bool = False,
                     object_id: Optional[str] = None, retry_count: int = 0,
                     error: Optional[ErrorType] = None) -> None:
        """
        Record a place operation outcome.
        
        Args:
            success: Whether the place was successful
            correct_zone: Whether the object was placed in the correct zone
            object_id: ID of the object being placed
            retry_count: Number of retries attempted
            error: Error type if operation failed
        """
        if success:
            self.successful_places += 1
            if correct_zone:
                self.correct_placements += 1
            else:
                self.incorrect_placements += 1
        else:
            self.failed_places += 1
        
        # Record in operation history
        record = OperationRecord(
            timestamp=time(),
            operation_type=OperationType.PLACE,
            success=success,
            object_id=object_id,
            error=error,
            retry_count=retry_count,
            correct_placement=correct_zone if success else None
        )
        self.operation_history.append(record)

    def record_collision(self, object_id: Optional[str] = None) -> None:
        """
        Record a collision event.
        
        Args:
            object_id: ID of the object involved in the collision (if any)
        """
        self.collisions += 1
        
        # Record in operation history
        record = OperationRecord(
            timestamp=time(),
            operation_type=OperationType.PICK,  # Collisions can occur during any movement
            success=False,
            object_id=object_id,
            error=ErrorType.COLLISION,
            retry_count=0
        )
        self.operation_history.append(record)

    def record_operation(self, operation: OperationRecord) -> None:
        """
        Record a complete operation record directly.
        
        Args:
            operation: The operation record to add to history
        """
        self.operation_history.append(operation)
        
        # Update counters based on operation type
        if operation.operation_type == OperationType.PICK:
            if operation.success:
                self.successful_picks += 1
            else:
                self.failed_picks += 1
                if operation.error == ErrorType.MISSED_PICK:
                    self.missed_picks += 1
        elif operation.operation_type == OperationType.PLACE:
            if operation.success:
                self.successful_places += 1
                if operation.correct_placement:
                    self.correct_placements += 1
                else:
                    self.incorrect_placements += 1
            else:
                self.failed_places += 1
        
        # Track collisions
        if operation.error == ErrorType.COLLISION:
            self.collisions += 1

    def start_timing(self) -> None:
        """Start timing the simulation cycle."""
        self.start_time = time()
        self.end_time = None

    def stop_timing(self) -> None:
        """Stop timing the simulation cycle."""
        self.end_time = time()

    def calculate_metrics(self, total_objects: int = 0, 
                         cycle_complete: bool = True) -> MetricsReport:
        """
        Calculate and return comprehensive metrics report.
        
        Args:
            total_objects: Total number of objects in the simulation
            cycle_complete: Whether the simulation cycle completed successfully
            
        Returns:
            MetricsReport with all calculated metrics
        """
        # Calculate total operations
        total_successful = self.successful_picks + self.successful_places
        total_failed = self.failed_picks + self.failed_places
        total_operations = total_successful + total_failed
        
        # Calculate success rate
        success_rate = (total_successful / total_operations * 100.0 
                       if total_operations > 0 else 0.0)
        
        # Calculate accuracy (correct placements / total placements)
        total_placements = self.correct_placements + self.incorrect_placements
        accuracy = (self.correct_placements / total_placements * 100.0
                   if total_placements > 0 else 0.0)
        
        # Calculate total time
        if self.start_time is not None:
            if self.end_time is not None:
                total_time = self.end_time - self.start_time
            else:
                total_time = time() - self.start_time
        else:
            total_time = 0.0
        
        # Calculate operations per second
        ops_per_second = (total_operations / total_time 
                         if total_time > 0 else 0.0)
        
        return MetricsReport(
            total_objects=total_objects,
            successful_operations=total_successful,
            failed_operations=total_failed,
            success_rate=success_rate,
            accuracy=accuracy,
            total_time=total_time,
            collisions=self.collisions,
            missed_picks=self.missed_picks,
            operations_per_second=ops_per_second,
            cycle_complete=cycle_complete,
            successful_picks=self.successful_picks,
            failed_picks=self.failed_picks,
            successful_places=self.successful_places,
            failed_places=self.failed_places,
            correct_placements=self.correct_placements,
            incorrect_placements=self.incorrect_placements
        )

    def reset(self) -> None:
        """Reset all metrics counters and history."""
        self.successful_picks = 0
        self.failed_picks = 0
        self.successful_places = 0
        self.failed_places = 0
        self.collisions = 0
        self.missed_picks = 0
        self.correct_placements = 0
        self.incorrect_placements = 0
        self.start_time = None
        self.end_time = None
        self.operation_history = []

    def save_to_history(self, report: MetricsReport) -> None:
        """
        Save a metrics report to the history.
        
        Args:
            report: The MetricsReport to save to history
        """
        self.metrics_history.append(report)

    def get_history(self) -> List[MetricsReport]:
        """
        Retrieve all historical metrics reports.
        
        Returns:
            List of all MetricsReport objects in history
        """
        return self.metrics_history.copy()

    def get_history_count(self) -> int:
        """
        Get the number of cycles stored in history.
        
        Returns:
            Number of MetricsReport objects in history
        """
        return len(self.metrics_history)

    def get_latest_report(self) -> Optional[MetricsReport]:
        """
        Get the most recent metrics report from history.
        
        Returns:
            The latest MetricsReport, or None if history is empty
        """
        if not self.metrics_history:
            return None
        return self.metrics_history[-1]

    def compare_with_previous(self, current_report: MetricsReport) -> Optional[dict]:
        """
        Compare a current report with the previous cycle's report.
        
        Args:
            current_report: The current MetricsReport to compare
            
        Returns:
            Dictionary with comparison metrics, or None if no previous report exists.
            Contains deltas for key metrics like success_rate, accuracy, etc.
        """
        if not self.metrics_history:
            return None
        
        previous = self.metrics_history[-1]
        
        return {
            'success_rate_delta': current_report.success_rate - previous.success_rate,
            'accuracy_delta': current_report.accuracy - previous.accuracy,
            'total_time_delta': current_report.total_time - previous.total_time,
            'operations_per_second_delta': current_report.operations_per_second - previous.operations_per_second,
            'collisions_delta': current_report.collisions - previous.collisions,
            'missed_picks_delta': current_report.missed_picks - previous.missed_picks,
            'previous_success_rate': previous.success_rate,
            'previous_accuracy': previous.accuracy,
            'current_success_rate': current_report.success_rate,
            'current_accuracy': current_report.accuracy
        }

    def get_average_metrics(self) -> Optional[dict]:
        """
        Calculate average metrics across all cycles in history.
        
        Returns:
            Dictionary with average values for key metrics, or None if history is empty
        """
        if not self.metrics_history:
            return None
        
        count = len(self.metrics_history)
        
        return {
            'avg_success_rate': sum(r.success_rate for r in self.metrics_history) / count,
            'avg_accuracy': sum(r.accuracy for r in self.metrics_history) / count,
            'avg_total_time': sum(r.total_time for r in self.metrics_history) / count,
            'avg_operations_per_second': sum(r.operations_per_second for r in self.metrics_history) / count,
            'avg_collisions': sum(r.collisions for r in self.metrics_history) / count,
            'avg_missed_picks': sum(r.missed_picks for r in self.metrics_history) / count,
            'total_cycles': count
        }

    def clear_history(self) -> None:
        """Clear all historical metrics reports."""
        self.metrics_history = []
