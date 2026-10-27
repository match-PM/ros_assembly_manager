"""Value objects used while resolving frame constraints."""

from geometry_msgs.msg import Vector3


class ConstraintRestriction:
    """Describe the constrained vector directions of a reference frame."""

    def __init__(
            self,
            frame_name: str,
            constraining_vectors: list[Vector3],
            vector_directions: list[str]):
        self.frame_name = frame_name
        self.constraining_vectors = constraining_vectors
        self.vector_directions = vector_directions

    def __str__(self):
        return (
            f'Frame: {self.frame_name}, '
            f'Constraining vectors: {self.constraining_vectors}, '
            f'Vector directions: {self.vector_directions}'
        )


class ConstraintRestrictionList:
    """Collection of frame constraint restrictions."""

    def __init__(self):
        self._restrictions: list[ConstraintRestriction] = []

    def get_list(self) -> list[ConstraintRestriction]:
        return self._restrictions

    def add_entry(self, restriction: ConstraintRestriction):
        self._restrictions.append(restriction)

    def add_list_to_list(self, other_list: 'ConstraintRestrictionList'):
        if not isinstance(other_list, ConstraintRestrictionList):
            raise TypeError('Expected a ConstraintRestrictionList instance.')
        self._restrictions.extend(other_list._restrictions)

    def __iter__(self):
        return iter(self._restrictions)

    def __len__(self):
        return len(self._restrictions)

    def __str__(self):
        return '\n'.join(str(restriction) for restriction in self)
