class AgentCtrl(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2022-06-22 04:50:33 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 15
        self.input_vars = ['']

    def move(self, ):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['X_h', 'X_v']
        """
        output = dict()
        if self.state == 0:
            if True:
                self.state = 1

                output["X_h"] = 5
                output["X_v"] = 4
        elif self.state == 1:
            if True:
                self.state = 2

                output["X_h"] = 5
                output["X_v"] = 3
        elif self.state == 2:
            if True:
                self.state = 3

                output["X_h"] = 5
                output["X_v"] = 2
        elif self.state == 3:
            if True:
                self.state = 4

                output["X_h"] = 5
                output["X_v"] = 1
        elif self.state == 4:
            if True:
                self.state = 5

                output["X_h"] = 5
                output["X_v"] = 0
        elif self.state == 5:
            if True:
                self.state = 6

                output["X_h"] = 4
                output["X_v"] = 0
        elif self.state == 6:
            if True:
                self.state = 7

                output["X_h"] = 3
                output["X_v"] = 0
        elif self.state == 7:
            if True:
                self.state = 8

                output["X_h"] = 2
                output["X_v"] = 0
        elif self.state == 8:
            if True:
                self.state = 9

                output["X_h"] = 1
                output["X_v"] = 0
        elif self.state == 9:
            if True:
                self.state = 10

                output["X_h"] = 0
                output["X_v"] = 0
        elif self.state == 10:
            if True:
                self.state = 11

                output["X_h"] = 0
                output["X_v"] = 1
        elif self.state == 11:
            if True:
                self.state = 12

                output["X_h"] = 0
                output["X_v"] = 2
        elif self.state == 12:
            if True:
                self.state = 13

                output["X_h"] = 0
                output["X_v"] = 3
        elif self.state == 13:
            if True:
                self.state = 14

                output["X_h"] = 0
                output["X_v"] = 4
        elif self.state == 14:
            if True:
                self.state = 14

                output["X_h"] = 0
                output["X_v"] = 4
        elif self.state == 15:
            if True:
                self.state = 0

                output["X_h"] = 5
                output["X_v"] = 5
        else:
            raise Exception("Unrecognized internal state: " + str(self.state))
        return output

    def _error(self, ):
        raise ValueError("Unrecognized input: " + ().format())