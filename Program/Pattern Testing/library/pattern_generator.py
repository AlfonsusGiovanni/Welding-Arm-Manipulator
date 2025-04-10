class Pattern_Type:
    DOT_TYPE = 0
    LINEAR_TYPE = 1
    CIRCULAR_TYPE = 2
    WAVE_TYPE = 3

class Pattern_Generator:
    def __init__(self):
        self.type = 0
        self.amplitude = 0
        self.cycle = 0

        self.iteration = 0

        self.pattern_type = Pattern_Type()

    def set_pattern(self, input_type, input_iteration):
        self.type = input_type
        self.iteration = 1/input_iteration

    def get_pattern_point(self, input_amplitude, input_counter):
        if self.type == self.pattern_type.DOT_TYPE:
            pass

        elif self.type == self.pattern_type.LINEAR_TYPE:
            return 0

        elif self.type == self.pattern_type.CIRCULAR_TYPE:
            pass

        elif self.type == self.pattern_type.WAVE_TYPE:
            self.amplitude = input_amplitude

            if input_counter % 2 == 0:
                return self.amplitude
            else:
                return 0