class Interpolation_Type:
    LINEAR = 0X01,
    POLYNOMIAL = 0x02
    SPLINE = 0x03

class Trajectory:
    def __init__(self):
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0

        self.point1 = 0
        self.point2 = 0
        self.point3 = 0
        self.point4 = 0

    def set_iteration_point(self, inputP1, inputP2, inputP3, inputP4):
        self.point1 = inputP1
        self.point2 = inputP2
        self.point3 = inputP3
        self.point4 = inputP4

    def calculate(self, input_mode, input_t):
        if input_mode == 0:
            point_range = self.point2 - self.point1

            if(self.point1 > 0):
                return (point_range * input_t) + self.point1
            elif(self.point1 < 0):
                return (point_range * input_t) - abs(self.point1)

        # USING POLYNOMIAL EQUATION
        elif input_mode == 1:
            self.a = pow((1-input_t), 3)
            self.b = 3*input_t*pow((1-input_t),2)
            self.c = 3*pow(input_t,2)*(1-input_t)
            self.d = pow(input_t,3)
            return (self.a*self.point1) + (self.b*self.point2) + (self.c*self.point3) + (self.d*self.point4)