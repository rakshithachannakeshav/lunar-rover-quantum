try:
    from scipy.ndimage import gaussian_filter
    print("scipy_ok")
except ImportError:
    print("scipy_missing")
