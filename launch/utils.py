import os

def expand(path: str) -> str:
    expanded_path = os.path.expanduser(path)
    # assert that the path exists
    assert os.path.exists(expanded_path), f"{expanded_path} does NOT exist!"
    return expanded_path