from scipy.stats import norm


M_PER_FT = 0.3048


def compute_sigma(volume_size: float, p_containment: float) -> float:
    """Compute scale of normal distribution.

    Compute the standard deviation that causes p_containment (fraction) of normal distribution samples to fall within a volume_size interval centered on the mean.

    :param volume_size: Size of mean-centered interval in which p_containment samples must fall.
    :param p_containment: Fraction of samples that must fall in volume_size interval.
    :return: Standard deviation (sigma) scale parameter of normal distribution.
    """
    return volume_size / 2 / norm.ppf(1 - (1 - p_containment) / 2)


def compute_volume_size(sigma: float, p_containment: float) -> float:
    return 2 * sigma * norm.ppf(1 - (1 - p_containment) / 2)
