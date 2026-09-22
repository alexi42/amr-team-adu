import rclpy
import smach
import threading
from .create_waypoints import CreateWaypoints
from .follow_waypoints import FollowWaypoints
from .goal_reached import GoalReached
from .goal_unreachable import GoalUnreachable

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
    sm = smach.StateMachine(outcomes=['orientation_reached', 'goal_unreachable'])

    # Add states to state machine
    with sm:
        smach.StateMachine.add(
            'CREATE_WAYPOINTS',
            CreateWaypoints(node, ROW=ROW, COL=COL, RESOLUTION=RESOLUTION),
            transitions={
                'create_waypoints': 'CREATE_WAYPOINTS',
                'driving_to_goal': 'FOLLOW_WAYPOINTS'
            }
        )

        smach.StateMachine.add(
            'FOLLOW_WAYPOINTS',
            FollowWaypoints(node, ROW=ROW, COL=COL, RESOLUTION=RESOLUTION),
            transitions={
                'driving_to_goal': 'FOLLOW_WAYPOINTS',
                'obstacle_encountered': 'CREATE_WAYPOINTS',
                'goal_reached': 'GOAL_REACHED',
                'create_waypoints': 'CREATE_WAYPOINTS',
                'goal_unreachable': 'GOAL_UNREACHABLE'
            }
        )

        smach.StateMachine.add(
            'GOAL_REACHED',
            GoalReached(node),
            transitions={
                'turning_to_given_orientation': 'GOAL_REACHED',
                'orientation_reached': 'orientation_reached'
            }
        )

        smach.StateMachine.add(
            'GOAL_UNREACHABLE',
            GoalUnreachable(node),
            transitions={
                'goal_unreachable': 'goal_unreachable'
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
