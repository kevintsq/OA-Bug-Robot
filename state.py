class AbstractState:
    """
    Abstract State with some default methods.
    All methods may be overridden by different states.
    """

    def __init__(self, robot):
        self.__robot = robot

    def get_robot(self):
        return self.__robot

    def transfer_to_next_state(self):
        if self.__robot.is_colliding_another_robot():
            self.transfer_when_colliding_another_robot()
        elif self.__robot.is_visiting_turning_point():
            self.transfer_when_not_following_wall()
        elif self.__robot.is_colliding_wall():
            self.transfer_when_colliding_wall()
        else:
            self.__robot.go_front()

    def transfer_when_colliding_wall(self):
        print(f"[{self.__robot}] Collides wall! Turning!")

    def transfer_when_colliding_another_robot(self):
        self.__robot.go_back()  # TODO
        print(f"[{self.__robot}] Collides another robot! Turning!")
        self.__robot.turn_right()  # TODO
        self.__robot.state = self.__robot.just_started_state

    def transfer_when_not_following_wall(self):
        pass

    def transfer_when_revisiting_places(self):
        self.__robot.print(f"[{self.__robot}] Finds this place has already been visited!")

    def __str__(self):
        return self.__class__.__name__

    def __hash__(self):
        return hash(self.__class__.__name__)

    def __eq__(self, other):
        return isinstance(other, AbstractState) and self.__class__.__name__ == other.__class__.__name__
