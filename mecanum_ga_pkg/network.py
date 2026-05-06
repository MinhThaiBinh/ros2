import numpy as np

class MecanumBrain:
    def __init__(self, input_size=12, hidden_size=16, output_size=2):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.weights_initialized = False
        self.w1 = np.zeros((input_size, hidden_size))
        self.b1 = np.zeros(hidden_size)
        self.w2 = np.zeros((hidden_size, output_size))
        self.b2 = np.zeros(output_size)

    def forward(self, inputs):
        h = np.tanh(np.dot(inputs, self.w1) + self.b1)
        outputs = np.tanh(np.dot(h, self.w2) + self.b2)
        return outputs 

    def set_genome(self, genome):
        # In ra để debug xem genome có rỗng không
        # print(f"DEBUG: Setting genome with length {len(genome)}")
        self.weights_initialized = True
        w1_end = self.input_size * self.hidden_size
        b1_end = w1_end + self.hidden_size
        w2_end = b1_end + (self.hidden_size * self.output_size)
        
        try:
            self.w1 = genome[0:w1_end].reshape(self.input_size, self.hidden_size)
            self.b1 = genome[w1_end:b1_end]
            self.w2 = genome[b1_end:w2_end].reshape(self.hidden_size, self.output_size)
            self.b2 = genome[w2_end:]
        except Exception as e:
            print(f"ERROR Reshaping genome: {e}")
            self.weights_initialized = False

    def get_genome(self):
        return np.concatenate([self.w1.flatten(), self.b1.flatten(), self.w2.flatten(), self.b2.flatten()])
