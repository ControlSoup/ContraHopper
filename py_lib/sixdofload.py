import numpy as np

# https://www.researchgate.net/publication/263965261_Six_component_force-torque_sensors_using_Gough-Stewart_platform_manipulators
# Six Component force torque sensors using stewart platform math

class LoadCell:
    def __init__(self, start, end):
        self.__start = np.array(start)
        self.__end = np.array(end)
        self.__vec = self.__start - self.__end


@dataclass
class SixDOFConfig:
    l1: LoadCell
    l2: LoadCell
    l3: LoadCell
    l4: LoadCell
    l5: LoadCell
    l6: LoadCell

    def from_arrays(
        l1: list[list, list],
        l2: list[list, list],
        l3: list[list, list],
        l4: list[list, list],
        l5: list[list, list],
        l6: list[list, list],
    ) -> "SixDOFConfig":
        return SixDOFConfig(
            l1 = LoadCell(start = l1[0], end = l1[1]),
            l2 = LoadCell(start = l2[0], end = l2[1]),
            l3 = LoadCell(start = l3[0], end = l3[1]),
            l4 = LoadCell(start = l4[0], end = l4[1]),
            l5 = LoadCell(start = l5[0], end = l5[1]),
            l6 = LoadCell(start = l6[0], end = l6[1]),
        )

    