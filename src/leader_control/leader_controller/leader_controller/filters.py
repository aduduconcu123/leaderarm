"""Optional first-order filtering of commanded targets."""


class EmaFilter:
    def __init__(self, alpha=1.0):
        if not 0.0 < alpha <= 1.0:
            raise ValueError('EMA alpha must be in (0, 1]')
        self.alpha = alpha
        self.value = None

    def reset(self, value):
        self.value = value

    def update(self, value):
        if self.value is None:
            self.value = value
        else:
            self.value += self.alpha * (value - self.value)
        return self.value
