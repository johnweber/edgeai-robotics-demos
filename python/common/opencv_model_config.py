class OpenCVModelConfig:
    """
    Shim class to duplicate some configuration to use OpenCV processing
    for certain video data flows.
    """
    count = 0
    def __init__(self, model_name):
        """
        Constructor of Open CV Model class.
        Args:
            model_name: the name of the OpenCV process we'll use
        """

        # No use TIDL
        self.enable_tidl = False

        # Set core number to 'something'
        self.core_number = 0

        # We'll patch in some code to check the task_type
        self.task_type = 'opencv'

        # Set Default values of some viz parameters
        self.alpha = 0.4
        self.viz_threshold = 0.5
        self.topN = 5

        self.path = None

        self.resize = [-1,-1]

        self.data_layout = None

        self.run_time = False

        self.reverse_channels = False

        # Our model name is the target OpenCV process we'll use.
        self.model_name = model_name