import ntcore
import logging

logging.basicConfig(level=logging.DEBUG)

class NetworkTable:
    def __init__(self):
        """
        Set up the connection to the robot and its NetworkTable.
        """
        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._table = self._inst.getTable("vision")
        self._distance = self._table.getDoubleTopic("x_distance").publish()
        self._horizontal_angle = self._table.getDoubleTopic("x_angle").publish()
        self._vertical_angle = self._table.getDoubleTopic("y_angle").publish()
        

        try:
            # Start the client and set the server IP
            self._inst.startClient4("vision client")
            self._inst.setServer("10.74.76.227", self._inst.kDefaultPort4)
            logging.info("Connected to server at 10.74.76.227")
        except Exception as e:
            logging.error(f"Failed to connect to the server: {e}")

    def send_data(self, distance: float, horizontal_angle: float, vertical_angle: float) -> None:
        self._distance.set(distance)
        self._horizontal_angle.set(horizontal_angle)
        self._vertical_angle.set(vertical_angle)
        logging.debug("x_angle: %.2f, y_angle: %.2f, distance: %.2f", horizontal_angle,vertical_angle, distance)

    def close(self):
        self._inst.stopClient()

    @property
    def instance(self):
        return self._inst

# Use this function to check connectivity
def check_connection():
    try:
        # Attempt to ping or connect
        network_table = NetworkTable()
        network_table.send_data(1,2,3)
        logging.info("Connection successful")
    except Exception as e:
        logging.error("Connection failed: %s", e)

if __name__ == "__main__":
    check_connection()