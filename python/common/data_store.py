#  Copyright (C) 2024 TechNexion Ltd.


"""
Common mutex-protected data store for passing data between inferrence pipes
and robot controller thread.
"""
import threading

class ObjectData:
    def __init__(self):
        self._mutex = threading.Lock()
        self._data = None
        self._is_valid = False

    def set(self, new_data):
        with self._mutex:
            self._data = new_data
            self._is_valid = True

    def clear(self):
        with self._mutex:
            self._data = None
            self._is_valid = False

    def get(self):
        with self._mutex:
            if(self._is_valid):
                return self._data
            else:
                return None

class DataStore:
    """
    """
    def __init__(self):
        """
        Constructor.
        """
        self.subject_position = ObjectData()
        self.patch_position_green = ObjectData()
        self.patch_position_blue = ObjectData()
        self.patch_position_orange = ObjectData()
        self.ball_color = ObjectData()




