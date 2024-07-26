from numpy import genfromtxt

def get_record(filename):
    data = None
    with open(filename, 'r') as file_handler:
        data = genfromtxt(file_handler, delimiter=',', names=True)
        print(f"load record successfully from file ({filename})")

    return data