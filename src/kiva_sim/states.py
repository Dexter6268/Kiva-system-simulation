from enum import IntEnum


class OrderStatus(IntEnum):
    """
    Enum for order status.
    """

    DONE = 0
    DOING = 1
    TODO = 2

    def __repr__(self):
        return self.name.lower()


class AgvStatus(IntEnum):
    """Enumeration for AGV operational states.

    This enum defines all possible states that an AGV can be in during warehouse
    operations, including order fulfillment, charging, and standby modes.

    Attributes:
        AVAILABLE: AGV is idle and ready for new order assignment.
        TO_SHELF: AGV is traveling from start position to target shelf location
            (includes lifting the shelf).
        TO_SELECT: AGV is traveling from shelf location to target workstation
            (includes waiting for sorting at workstation).
        WAITING_TO_SELECT: AGV is waiting at shelf location because target
            workstation is occupied.
        SELECTING: AGV is at workstation waiting for order sorting/picking process.
        RETURN_SHELF: AGV is returning shelf from workstation to original location
            (includes placing the shelf down).
        TO_CHARGE: AGV has completed orders and is traveling from shelf location
            to charging station.
        WAITING_TO_CHARGE: AGV is waiting at shelf location because all charging
            stations are occupied.
        CHARGING: AGV is at charging station replenishing battery.
        BACK_TO_START: AGV is returning to start position after completing orders
            or charging.
        ARRIVED_AT_START: AGV has reached start position and the position is marked
            as non-traversable for path refresh.
        WAITING_AT_START: AGV is on standby at start position with no assigned
            orders and sufficient battery level.
    """

    AVAILABLE = 0
    TO_SHELF = 1
    TO_SELECT = 2
    WAITING_TO_SELECT = 3
    SELECTING = 4
    RETURN_SHELF = 5
    TO_CHARGE = 6
    WAITING_TO_CHARGE = 7
    CHARGING = 8
    BACK_TO_START = 9
    ARRIVED_AT_START = 10
    WAITING_AT_START = 11

    def __repr__(self):
        return self.name.lower()


class Direction(IntEnum):
    """
    Enum for representing the four cardinal directions.
    """

    UP = 0
    RIGHT = 90
    DOWN = 180
    LEFT = 270

    def __repr__(self):
        return self.name.lower()
