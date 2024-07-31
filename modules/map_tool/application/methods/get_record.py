from numpy import genfromtxt

def get_record(filename):
    data = None
    with open(filename, 'r') as file_handler:
        data = genfromtxt(file_handler, delimiter=',', names=True)
        print(f"loaded record successfully from file ({filename})")

    return data

def get_strange_record(filename, converter):
    data = {"x": [], "y": []}
    with open(filename, 'r') as file_handler:
        lines = file_handler.readlines()
    lines = lines[1:]
    for line in lines:
        lat, lon, _ = line.replace(',', '.').split(';')
        x, y = converter.lonlat_to_utm(lon, lat)
        data["x"].append(x)
        data["y"].append(y)

    return data
