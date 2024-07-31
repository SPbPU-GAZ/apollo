import os
import cv2
import json
from application.methods.tile_api import get_tile_img, get_tile_grid, get_tile_bounds, create_tile_xyz


def get_tile_center(tile, converter):
    tb = get_tile_bounds(tile)
    _n, _s, _w, _e = tb.north, tb.south, tb.west, tb.east
    Px, Py = converter.lonlat_to_utm((_w + _e) / 2.0, (_n + _s) / 2.0)
    return [Px, Py]


def calculate_my_bounds(tile, converter):
    x, y, z = tile

    tile_ul     = create_tile_xyz(x - 1, y - 1, z)
    tile_ur     = create_tile_xyz(x + 1, y - 1, z)
    tile_dl     = create_tile_xyz(x - 1, y + 1, z)
    tile_dr     = create_tile_xyz(x + 1, y + 1, z)

    c_x, c_y = get_tile_center(tile, converter)

    ul_x, ul_y = get_tile_center(tile_ul, converter)
    ur_x, ur_y = get_tile_center(tile_ur, converter)
    dl_x, dl_y = get_tile_center(tile_dl, converter)
    dr_x, dr_y = get_tile_center(tile_dr, converter)

    Ax, Ay = (ul_x + c_x)/2, (ul_y + c_y)/2
    Bx, By = (ur_x + c_x)/2, (ur_y + c_y)/2
    Cx, Cy = (dr_x + c_x)/2, (dr_y + c_y)/2
    Dx, Dy = (dl_x + c_x)/2, (dl_y + c_y)/2

    return [Ax, Ay, Bx, By, Cx, Cy, Dx, Dy]


def load_tile(tile, scenario):
    converter, service, rtv = scenario.get_converter(), scenario.get_map_api_service(), scenario.get_rtv()
    
    x, y, z = tile
    img_file = os.path.join(rtv.TMP_DIR, f"{service.name()}_{x}_{y}_{z}.png")
    cords_file = os.path.join(rtv.TMP_DIR, f"{service.name()}_{x}_{y}_{z}.json")

    if not os.path.exists(img_file):
        print(f"downloading {x}/{y}/{z} tile")
        img = get_tile_img(tile, service)
        img = img[..., :3]
        cv2.imwrite(img_file, img)

    if not os.path.exists(cords_file):
        bounds = calculate_my_bounds(tile, converter)
        with open(cords_file, "w") as f:
            json.dump(bounds, f)

    with open(cords_file, "r") as f:
        bounds = json.load(f)
        img = cv2.imread(img_file)

    return img, bounds


def get_cluster(cords, scenario):
    converter = scenario.get_converter()

    Ax, Ay = cords[0]
    Cx, Cy = cords[1]
    
    lonA, latA = converter.utm_to_lonlat(Ax, Ay)
    lonC, latC = converter.utm_to_lonlat(Cx, Cy)

    tiles, res = get_tile_grid(lonA, latC, lonC, latA, 19), []
    for tile in tiles:
        x, y, z = tile
        name = f"{x}_{y}_{z}"
        img, bounds = load_tile(tile, scenario)
        res.append([name, img, bounds])
    return res
