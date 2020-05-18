import numpy as np
import matplotlib.pyplot as plt

# X = np.arange(0, 5, 0.1)
# Z = [3 + 5 * x for x in X]
# Y = [np.random.normal(z, 0.5) for z in Z]
# plt.plot(X, Y, 'ro')
# plt.show()

def sampling_points_visualization(x,y):
    plt.plot(x,y,'ro')
    plt.show()
def linear_regression(x,y):
    N = len(x)
    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum(x ** 2)
    sumxy = sum(x * y)
    A = np.mat([[N, sumx], [sumx, sumx2]])
    b = np.array([sumy, sumxy])
    return np.linalg.solve(A, b)

def main():
    x=[]
    y=[]
    sampling_points_visualization(x,y)
    a0,a1 = linear_regression(x,y)
if __name__ == "__main__":
    main()


