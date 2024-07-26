import pyproj
import os
import cv2
import json
from application.methods.tile_api import get_tile_img, get_tile_grid, get_tile_bounds, create_tile_xyz


def calculate_my_bounds(tile, converter): #evety corner using only one point (like A or C for nearest points)
    x, y, z = tile
    #tile_up     = create_tile_xyz(x, y - 1, z)
    tile_down   = create_tile_xyz(x, y + 1, z)
    #tile_left   = create_tile_xyz(x - 1, y, z)
    tile_right  = create_tile_xyz(x + 1, y, z)

    # tile_ul     = create_tile_xyz(x - 1, y - 1, z)
    # tile_ur     = create_tile_xyz(x + 1, y - 1, z)
    # tile_dl     = create_tile_xyz(x - 1, y + 1, z)
    tile_dr     = create_tile_xyz(x + 1, y + 1, z)

    #Ax, Ay, Cx, Cy = get_utm_bbox(tile, converter)

    # uAx, uAy, uCx, uCy = get_utm_bbox(tile_up, converter)
    # dAx, dAy, dCx, dCy = get_utm_bbox(tile_down, converter)
    # lAx, lAy, lCx, lCy = get_utm_bbox(tile_left, converter)
    # rAx, rAy, rCx, rCy = get_utm_bbox(tile_right, converter)

    # ulAx, ulAy, ulCx, ulCy = get_utm_bbox(tile_ul, converter)
    # urAx, urAy, urCx, urCy = get_utm_bbox(tile_ur, converter)
    # dlAx, dlAy, dlCx, dlCy = get_utm_bbox(tile_dl, converter)
    # drAx, drAy, drCx, drCy = get_utm_bbox(tile_dr, converter)

    # Ax = (_Ax + uAx + ulCx + lCx) / 4
    # Ay = (_Ay + uCy + ulCy + lAy) / 4
    # Cx = (_Cx + dCx + drAx + rAx) / 4
    # Cy = (_Cy + dAy + drAy + rCy) / 4 

    #_, _, Bx, By = get_utm_bbox(tile_up, converter) #upper C
    #Dx, Dy, _, _ = get_utm_bbox(tile_down, converter) #lower A

    #print(Cx - Ax, Ay - Cy)

    Ax, Ay, _, _ = get_utm_bbox(tile, converter)
    Bx, By, _, _ = get_utm_bbox(tile_right, converter)
    Cx, Cy, _, _ = get_utm_bbox(tile_dr, converter)
    Dx, Dy, _, _ = get_utm_bbox(tile_down, converter)
    
    return [Ax, Ay, Bx, By, Cx, Cy, Dx, Dy]
    


def get_utm_bbox(tile, converter):
    tb = get_tile_bounds(tile)
    
    _n, _s, _w, _e = tb.north, tb.south, tb.west, tb.east
    
    bb_Ax, bb_Cy = converter.lonlat_to_utm(_w, _s)
    bb_Cx, bb_Ay = converter.lonlat_to_utm(_e, _n)

    return [bb_Ax, bb_Ay, bb_Cx, bb_Cy]


def load_tile(tile, scenario):
    converter, service, cfg = scenario.get_converter(), scenario.get_map_api_service(), scenario.get_cfg()
    
    x, y, z = tile
    img_file = os.path.join(cfg.TMP_DIR, f"{service.name()}_{x}_{y}_{z}.png")
    cords_file = os.path.join(cfg.TMP_DIR, f"{service.name()}_{x}_{y}_{z}.json")

    if not os.path.exists(img_file):
        img = get_tile_img(tile, service)
        img = img[..., :3]
        cv2.imwrite(img_file, img)


    if  True:#not os.path.exists(cords_file):
        bounds = calculate_my_bounds(tile, converter) #get_utm_bbox(tile, converter)
        #bb_Ax, bb_Ay, bb_Cx, bb_Cy
        #bounds = [[bb_Ax, bb_Ay], [bb_Cx, bb_Cy]]

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

    tiles = get_tile_grid(lonA, latC, lonC, latA, 19)

    res = []
    for tile in tiles:
        img, bounds = load_tile(tile, scenario)
        res.append([img , bounds])
    return res
