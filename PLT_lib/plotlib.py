import matplotlib.pyplot as plt
import math

class PlotLib():
    """
    pyplot 
    """    
    def __init__(self, title = "Test", xlabel="x[m]", ylabel="y[m]"):
        plt.cla()
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        # plt.axis("equal")
        # plt.grid(True)
        
    @staticmethod
    def draw(x, y, color="r", label="Test"):
        plt.plot(x, y, color, label=label)

    @staticmethod
    def set_legend():
        plt.legend()

    @staticmethod
    def sleep(dt):
        plt.pause(dt)

    @staticmethod
    def show():   
        plt.axis("equal")
        plt.grid(True) 
        plt.show()

    @staticmethod
    def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                PlotLib.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)