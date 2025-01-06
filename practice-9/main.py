import numpy as np
import unittest


# функция подсчета суммы главной диагонали матрицы
def calc_diagonal_sum(matrix: np.array) -> np.ndarray:
    # Вычисляем сумму элементов главной диагонали
    return np.trace(matrix)


# unit тесты
class Tests(unittest.TestCase):
    def test1(self):
        matrix = np.array([[1, 2],
                           [3, 4]])
        self.assertEqual(calc_diagonal_sum(matrix), 5)  # 1 + 4 = 5

    def test2(self):
        matrix = np.array([[10, 2, 3],
                           [4, 20, 6],
                           [7, 8, 30]])
        self.assertEqual(calc_diagonal_sum(matrix), 60)  # 10 + 20 + 30 = 60

    def test3(self):
        matrix = np.array([[42]])
        self.assertEqual(calc_diagonal_sum(matrix), 42)  # Единственный элемент

    def test4(self):
        matrix = np.zeros((3, 3), dtype=int)
        self.assertEqual(calc_diagonal_sum(matrix), 0)  # Сумма диагонали нулевой матрицы

    def test5(self):
        matrix = np.array([[1, 2, 3],
                           [4, 5, 6]])
        self.assertEqual(calc_diagonal_sum(matrix), 6)


if __name__ == "__main__":
    unittest.main()
