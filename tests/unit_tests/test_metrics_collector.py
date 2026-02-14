"""Unit tests for MetricsCollector class."""

import pytest
from time import sleep
from robotic_sorting.metrics_collector import (
    MetricsCollector, OperationRecord, MetricsReport
)
from robotic_sorting.models import OperationType, ErrorType


class TestMetricsCollector:
    """Tests for MetricsCollector class."""

    def test_initialization(self):
        """Test that metrics collector initializes with zero counters."""
        collector = MetricsCollector()
        assert collector.successful_picks == 0
        assert collector.failed_picks == 0
        assert collector.successful_places == 0
        assert collector.failed_places == 0
        assert collector.collisions == 0
        assert collector.missed_picks == 0
        assert collector.correct_placements == 0
        assert collector.incorrect_placements == 0
        assert collector.start_time is None
        assert collector.end_time is None
        assert len(collector.operation_history) == 0

    def test_record_successful_pick(self):
        """Test recording a successful pick operation."""
        collector = MetricsCollector()
        collector.record_pick(success=True, object_id="obj1")
        
        assert collector.successful_picks == 1
        assert collector.failed_picks == 0
        assert collector.missed_picks == 0
        assert len(collector.operation_history) == 1
        
        record = collector.operation_history[0]
        assert record.operation_type == OperationType.PICK
        assert record.success is True
        assert record.object_id == "obj1"
        assert record.error is None

    def test_record_failed_pick(self):
        """Test recording a failed pick operation."""
        collector = MetricsCollector()
        collector.record_pick(success=False, missed=True, object_id="obj2", retry_count=2)
        
        assert collector.successful_picks == 0
        assert collector.failed_picks == 1
        assert collector.missed_picks == 1
        assert len(collector.operation_history) == 1
        
        record = collector.operation_history[0]
        assert record.operation_type == OperationType.PICK
        assert record.success is False
        assert record.object_id == "obj2"
        assert record.error == ErrorType.MISSED_PICK
        assert record.retry_count == 2

    def test_record_successful_place_correct_zone(self):
        """Test recording a successful place in correct zone."""
        collector = MetricsCollector()
        collector.record_place(success=True, correct_zone=True, object_id="obj1")
        
        assert collector.successful_places == 1
        assert collector.failed_places == 0
        assert collector.correct_placements == 1
        assert collector.incorrect_placements == 0
        assert len(collector.operation_history) == 1
        
        record = collector.operation_history[0]
        assert record.operation_type == OperationType.PLACE
        assert record.success is True
        assert record.object_id == "obj1"
        assert record.correct_placement is True

    def test_record_successful_place_incorrect_zone(self):
        """Test recording a successful place in incorrect zone."""
        collector = MetricsCollector()
        collector.record_place(success=True, correct_zone=False, object_id="obj1")
        
        assert collector.successful_places == 1
        assert collector.correct_placements == 0
        assert collector.incorrect_placements == 1

    def test_record_failed_place(self):
        """Test recording a failed place operation."""
        collector = MetricsCollector()
        collector.record_place(success=False, object_id="obj3", 
                              error=ErrorType.ZONE_FULL, retry_count=1)
        
        assert collector.successful_places == 0
        assert collector.failed_places == 1
        assert len(collector.operation_history) == 1
        
        record = collector.operation_history[0]
        assert record.operation_type == OperationType.PLACE
        assert record.success is False
        assert record.error == ErrorType.ZONE_FULL
        assert record.retry_count == 1

    def test_record_collision(self):
        """Test recording a collision event."""
        collector = MetricsCollector()
        collector.record_collision(object_id="obj4")
        
        assert collector.collisions == 1
        assert len(collector.operation_history) == 1
        
        record = collector.operation_history[0]
        assert record.error == ErrorType.COLLISION
        assert record.object_id == "obj4"

    def test_record_operation_directly(self):
        """Test recording an operation record directly."""
        collector = MetricsCollector()
        operation = OperationRecord(
            timestamp=1234567890.0,
            operation_type=OperationType.PICK,
            success=True,
            object_id="obj5"
        )
        collector.record_operation(operation)
        
        assert collector.successful_picks == 1
        assert len(collector.operation_history) == 1
        assert collector.operation_history[0] == operation

    def test_timing_tracking(self):
        """Test start and stop timing functionality."""
        collector = MetricsCollector()
        
        collector.start_timing()
        assert collector.start_time is not None
        assert collector.end_time is None
        
        start = collector.start_time
        sleep(0.01)  # Small delay
        
        collector.stop_timing()
        assert collector.end_time is not None
        assert collector.end_time > start

    def test_calculate_metrics_empty(self):
        """Test calculating metrics with no operations."""
        collector = MetricsCollector()
        collector.start_timing()
        collector.stop_timing()
        
        report = collector.calculate_metrics(total_objects=10)
        
        assert report.total_objects == 10
        assert report.successful_operations == 0
        assert report.failed_operations == 0
        assert report.success_rate == 0.0
        assert report.accuracy == 0.0
        assert report.collisions == 0
        assert report.missed_picks == 0
        assert report.cycle_complete is True

    def test_calculate_metrics_with_operations(self):
        """Test calculating metrics with various operations."""
        collector = MetricsCollector()
        collector.start_timing()
        
        # Record some operations
        collector.record_pick(success=True, object_id="obj1")
        collector.record_pick(success=True, object_id="obj2")
        collector.record_pick(success=False, missed=True, object_id="obj3")
        collector.record_place(success=True, correct_zone=True, object_id="obj1")
        collector.record_place(success=True, correct_zone=True, object_id="obj2")
        collector.record_collision(object_id="obj4")
        
        sleep(0.01)  # Small delay for timing
        collector.stop_timing()
        
        report = collector.calculate_metrics(total_objects=3)
        
        assert report.total_objects == 3
        assert report.successful_operations == 4  # 2 picks + 2 places
        assert report.failed_operations == 1  # 1 failed pick
        assert report.success_rate == 80.0  # 4/5 * 100
        assert report.accuracy == 100.0  # 2/2 correct placements
        assert report.collisions == 1
        assert report.missed_picks == 1
        assert report.total_time > 0
        assert report.operations_per_second > 0

    def test_calculate_success_rate(self):
        """Test success rate calculation."""
        collector = MetricsCollector()
        
        # 3 successful, 1 failed = 75% success rate
        collector.record_pick(success=True)
        collector.record_pick(success=True)
        collector.record_place(success=True, correct_zone=True)
        collector.record_pick(success=False, missed=True)
        
        report = collector.calculate_metrics()
        assert report.success_rate == 75.0

    def test_calculate_accuracy(self):
        """Test accuracy calculation for placements."""
        collector = MetricsCollector()
        
        # 2 correct, 1 incorrect = 66.67% accuracy
        collector.record_place(success=True, correct_zone=True)
        collector.record_place(success=True, correct_zone=True)
        collector.record_place(success=True, correct_zone=False)
        
        report = collector.calculate_metrics()
        assert abs(report.accuracy - 66.67) < 0.1

    def test_reset(self):
        """Test resetting all metrics."""
        collector = MetricsCollector()
        
        # Add some data
        collector.start_timing()
        collector.record_pick(success=True)
        collector.record_place(success=True, correct_zone=True)
        collector.record_collision()
        collector.stop_timing()
        
        # Reset
        collector.reset()
        
        # Verify everything is cleared
        assert collector.successful_picks == 0
        assert collector.failed_picks == 0
        assert collector.successful_places == 0
        assert collector.failed_places == 0
        assert collector.collisions == 0
        assert collector.missed_picks == 0
        assert collector.correct_placements == 0
        assert collector.incorrect_placements == 0
        assert collector.start_time is None
        assert collector.end_time is None
        assert len(collector.operation_history) == 0

    def test_multiple_operations_tracking(self):
        """Test tracking multiple operations of different types."""
        collector = MetricsCollector()
        
        # Simulate a sequence of operations
        for i in range(5):
            collector.record_pick(success=True, object_id=f"obj{i}")
            collector.record_place(success=True, correct_zone=True, object_id=f"obj{i}")
        
        assert collector.successful_picks == 5
        assert collector.successful_places == 5
        assert collector.correct_placements == 5
        assert len(collector.operation_history) == 10

    def test_operation_history_order(self):
        """Test that operation history maintains chronological order."""
        collector = MetricsCollector()
        
        collector.record_pick(success=True, object_id="obj1")
        collector.record_place(success=True, correct_zone=True, object_id="obj1")
        collector.record_pick(success=True, object_id="obj2")
        
        assert len(collector.operation_history) == 3
        assert collector.operation_history[0].operation_type == OperationType.PICK
        assert collector.operation_history[0].object_id == "obj1"
        assert collector.operation_history[1].operation_type == OperationType.PLACE
        assert collector.operation_history[1].object_id == "obj1"
        assert collector.operation_history[2].operation_type == OperationType.PICK
        assert collector.operation_history[2].object_id == "obj2"


class TestMetricsHistory:
    """Tests for metrics history functionality."""

    def test_initialization_includes_history(self):
        """Test that metrics collector initializes with empty history."""
        collector = MetricsCollector()
        assert len(collector.metrics_history) == 0
        assert collector.get_history_count() == 0

    def test_save_to_history(self):
        """Test saving a metrics report to history."""
        collector = MetricsCollector()
        
        # Create a sample report
        report = MetricsReport(
            total_objects=10,
            successful_operations=8,
            failed_operations=2,
            success_rate=80.0,
            accuracy=90.0,
            total_time=5.0,
            collisions=1,
            missed_picks=1,
            operations_per_second=2.0,
            cycle_complete=True
        )
        
        collector.save_to_history(report)
        
        assert collector.get_history_count() == 1
        assert collector.get_history()[0] == report

    def test_save_multiple_reports_to_history(self):
        """Test saving multiple reports to history."""
        collector = MetricsCollector()
        
        # Create and save multiple reports
        for i in range(5):
            report = MetricsReport(
                total_objects=10 + i,
                successful_operations=8,
                failed_operations=2,
                success_rate=80.0,
                accuracy=90.0,
                total_time=5.0 + i,
                collisions=i,
                missed_picks=1,
                operations_per_second=2.0,
                cycle_complete=True
            )
            collector.save_to_history(report)
        
        assert collector.get_history_count() == 5
        history = collector.get_history()
        assert len(history) == 5
        # Verify order is maintained
        assert history[0].total_objects == 10
        assert history[4].total_objects == 14

    def test_get_history_returns_copy(self):
        """Test that get_history returns a copy, not the original list."""
        collector = MetricsCollector()
        
        report = MetricsReport(
            total_objects=10,
            successful_operations=8,
            failed_operations=2,
            success_rate=80.0,
            accuracy=90.0,
            total_time=5.0,
            collisions=1,
            missed_picks=1,
            operations_per_second=2.0,
            cycle_complete=True
        )
        collector.save_to_history(report)
        
        history = collector.get_history()
        history.append(report)  # Modify the returned list
        
        # Original should be unchanged
        assert collector.get_history_count() == 1

    def test_get_latest_report_empty_history(self):
        """Test getting latest report when history is empty."""
        collector = MetricsCollector()
        assert collector.get_latest_report() is None

    def test_get_latest_report(self):
        """Test getting the most recent report from history."""
        collector = MetricsCollector()
        
        # Add multiple reports
        for i in range(3):
            report = MetricsReport(
                total_objects=10 + i,
                successful_operations=8,
                failed_operations=2,
                success_rate=80.0 + i,
                accuracy=90.0,
                total_time=5.0,
                collisions=1,
                missed_picks=1,
                operations_per_second=2.0,
                cycle_complete=True
            )
            collector.save_to_history(report)
        
        latest = collector.get_latest_report()
        assert latest is not None
        assert latest.total_objects == 12
        assert latest.success_rate == 82.0

    def test_compare_with_previous_no_history(self):
        """Test comparison when there's no previous report."""
        collector = MetricsCollector()
        
        current_report = MetricsReport(
            total_objects=10,
            successful_operations=8,
            failed_operations=2,
            success_rate=80.0,
            accuracy=90.0,
            total_time=5.0,
            collisions=1,
            missed_picks=1,
            operations_per_second=2.0,
            cycle_complete=True
        )
        
        comparison = collector.compare_with_previous(current_report)
        assert comparison is None

    def test_compare_with_previous(self):
        """Test comparing current report with previous cycle."""
        collector = MetricsCollector()
        
        # Add a previous report
        previous_report = MetricsReport(
            total_objects=10,
            successful_operations=7,
            failed_operations=3,
            success_rate=70.0,
            accuracy=85.0,
            total_time=6.0,
            collisions=2,
            missed_picks=2,
            operations_per_second=1.67,
            cycle_complete=True
        )
        collector.save_to_history(previous_report)
        
        # Create current report
        current_report = MetricsReport(
            total_objects=10,
            successful_operations=8,
            failed_operations=2,
            success_rate=80.0,
            accuracy=90.0,
            total_time=5.0,
            collisions=1,
            missed_picks=1,
            operations_per_second=2.0,
            cycle_complete=True
        )
        
        comparison = collector.compare_with_previous(current_report)
        
        assert comparison is not None
        assert comparison['success_rate_delta'] == 10.0
        assert comparison['accuracy_delta'] == 5.0
        assert comparison['total_time_delta'] == -1.0
        assert abs(comparison['operations_per_second_delta'] - 0.33) < 0.01
        assert comparison['collisions_delta'] == -1
        assert comparison['missed_picks_delta'] == -1
        assert comparison['previous_success_rate'] == 70.0
        assert comparison['previous_accuracy'] == 85.0
        assert comparison['current_success_rate'] == 80.0
        assert comparison['current_accuracy'] == 90.0

    def test_get_average_metrics_empty_history(self):
        """Test getting average metrics when history is empty."""
        collector = MetricsCollector()
        assert collector.get_average_metrics() is None

    def test_get_average_metrics_single_report(self):
        """Test getting average metrics with a single report."""
        collector = MetricsCollector()
        
        report = MetricsReport(
            total_objects=10,
            successful_operations=8,
            failed_operations=2,
            success_rate=80.0,
            accuracy=90.0,
            total_time=5.0,
            collisions=1,
            missed_picks=1,
            operations_per_second=2.0,
            cycle_complete=True
        )
        collector.save_to_history(report)
        
        averages = collector.get_average_metrics()
        
        assert averages is not None
        assert averages['avg_success_rate'] == 80.0
        assert averages['avg_accuracy'] == 90.0
        assert averages['avg_total_time'] == 5.0
        assert averages['avg_operations_per_second'] == 2.0
        assert averages['avg_collisions'] == 1.0
        assert averages['avg_missed_picks'] == 1.0
        assert averages['total_cycles'] == 1

    def test_get_average_metrics_multiple_reports(self):
        """Test getting average metrics across multiple cycles."""
        collector = MetricsCollector()
        
        # Add multiple reports with varying metrics
        reports_data = [
            (80.0, 90.0, 5.0, 2.0, 1, 1),
            (85.0, 95.0, 4.5, 2.2, 0, 0),
            (75.0, 85.0, 5.5, 1.8, 2, 2),
        ]
        
        for success_rate, accuracy, time, ops_per_sec, collisions, missed in reports_data:
            report = MetricsReport(
                total_objects=10,
                successful_operations=8,
                failed_operations=2,
                success_rate=success_rate,
                accuracy=accuracy,
                total_time=time,
                collisions=collisions,
                missed_picks=missed,
                operations_per_second=ops_per_sec,
                cycle_complete=True
            )
            collector.save_to_history(report)
        
        averages = collector.get_average_metrics()
        
        assert averages is not None
        assert abs(averages['avg_success_rate'] - 80.0) < 0.1
        assert abs(averages['avg_accuracy'] - 90.0) < 0.1
        assert abs(averages['avg_total_time'] - 5.0) < 0.1
        assert abs(averages['avg_operations_per_second'] - 2.0) < 0.1
        assert abs(averages['avg_collisions'] - 1.0) < 0.1
        assert abs(averages['avg_missed_picks'] - 1.0) < 0.1
        assert averages['total_cycles'] == 3

    def test_clear_history(self):
        """Test clearing all historical metrics."""
        collector = MetricsCollector()
        
        # Add some reports
        for i in range(3):
            report = MetricsReport(
                total_objects=10,
                successful_operations=8,
                failed_operations=2,
                success_rate=80.0,
                accuracy=90.0,
                total_time=5.0,
                collisions=1,
                missed_picks=1,
                operations_per_second=2.0,
                cycle_complete=True
            )
            collector.save_to_history(report)
        
        assert collector.get_history_count() == 3
        
        collector.clear_history()
        
        assert collector.get_history_count() == 0
        assert collector.get_latest_report() is None
        assert collector.get_average_metrics() is None

    def test_reset_does_not_clear_history(self):
        """Test that reset() clears current metrics but preserves history."""
        collector = MetricsCollector()
        
        # Add a report to history
        report = MetricsReport(
            total_objects=10,
            successful_operations=8,
            failed_operations=2,
            success_rate=80.0,
            accuracy=90.0,
            total_time=5.0,
            collisions=1,
            missed_picks=1,
            operations_per_second=2.0,
            cycle_complete=True
        )
        collector.save_to_history(report)
        
        # Add some current metrics
        collector.record_pick(success=True)
        collector.record_place(success=True, correct_zone=True)
        
        # Reset current metrics
        collector.reset()
        
        # Current metrics should be cleared
        assert collector.successful_picks == 0
        assert collector.successful_places == 0
        
        # History should be preserved
        assert collector.get_history_count() == 1
        assert collector.get_latest_report() == report

    def test_workflow_multiple_cycles(self):
        """Test a complete workflow with multiple simulation cycles."""
        collector = MetricsCollector()
        
        # Simulate 3 cycles
        for cycle in range(3):
            # Start timing
            collector.start_timing()
            
            # Record operations
            for i in range(5):
                collector.record_pick(success=True, object_id=f"obj{i}")
                collector.record_place(success=True, correct_zone=True, object_id=f"obj{i}")
            
            # Stop timing
            sleep(0.01)
            collector.stop_timing()
            
            # Calculate and save metrics
            report = collector.calculate_metrics(total_objects=5, cycle_complete=True)
            collector.save_to_history(report)
            
            # Reset for next cycle
            collector.reset()
        
        # Verify history
        assert collector.get_history_count() == 3
        
        # All cycles should have 100% success rate
        history = collector.get_history()
        for report in history:
            assert report.success_rate == 100.0
            assert report.accuracy == 100.0
        
        # Check averages
        averages = collector.get_average_metrics()
        assert averages['avg_success_rate'] == 100.0
        assert averages['total_cycles'] == 3
