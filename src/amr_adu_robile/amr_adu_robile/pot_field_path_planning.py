import rclpy
import smach
import threading
from .create_waypoints import CreateWaypoints
from .follow_waypoints import FollowWaypoints
from .goal_reached import GoalReached

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Grid size
ROW = 800
COL = 800
RESOLUTION = 0.25


def main(args=None):
    """Main function to initialise and execute state machine."""
    rclpy.init(args=args)

    node = rclpy.create_node('state_machine')
    node.START = None
    node.start_lock = threading.Lock()
    node.DESTINATION = None
    node.dest_lock = threading.Lock()

    # Create state machine
    sm = smach.StateMachine(outcomes=['orientation_reached', 'error'])

    # Add states to state machine
    with sm:
        smach.StateMachine.add(
            'CREATE WAYPOINTS',
            CreateWaypoints(node, ROW=ROW, COL=COL, RESOLUTION=RESOLUTION),
            transitions={
                'create_waypoints': 'CREATE WAYPOINTS',
                'driving_to_goal': 'FOLLOW WAYPOINTS'
            }
        )

        smach.StateMachine.add(
            'FOLLOW WAYPOINTS',
            FollowWaypoints(node, ROW=ROW, COL=COL, RESOLUTION=RESOLUTION),
            transitions={
                'driving_to_goal': 'FOLLOW WAYPOINTS',
                'obstacle_encountered': 'CREATE WAYPOINTS',
                'goal_reached': 'GOAL REACHED',
                'create_waypoints': 'CREATE WAYPOINTS'
            }
        )

        smach.StateMachine.add(
            'GOAL REACHED',
            GoalReached(node),
            transitions={
                'turning_to_given_orientation': 'GOAL REACHED',
                'orientation_reached': 'orientation_reached'
            }
        )

    # multihreaded approach because sm.execute() blocks the whole thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Execute state machine in a separate thread
    state_thread = threading.Thread(target=sm.execute)
    state_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
