import requests
import uuid
import io
import os
import numpy as np
from PIL import Image
import mercantile as mt

USER_AGENT = "contextily-" + uuid.uuid4().hex
#https://github.com/darribas/contextily/blob/9a8b652768365b24fbce912b71737f4fa6b3295c/contextily/tile.py#L422


def _retryer(tile_url, wait, max_retries):
    try:
        request = requests.get(tile_url, headers={"user-agent": USER_AGENT})
        request.raise_for_status()
    except requests.HTTPError:
        if request.status_code == 404:
            raise requests.HTTPError(
                "Tile URL resulted in a 404 error. "
                "Double-check your tile url:\n{}".format(tile_url)
            )
        elif request.status_code == 104:
            if max_retries > 0:
                os.wait(wait)
                max_retries -= 1
                request = _retryer(tile_url, wait, max_retries)
            else:
                raise requests.HTTPError("Connection reset by peer too many times.")
    return request


def _fetch_tile(tile_url, wait, max_retries):
    request = _retryer(tile_url, wait, max_retries)
    with io.BytesIO(request.content) as image_stream:
        image = Image.open(image_stream).convert("RGB")
        array = np.asarray(image)
        image.close()
    return array


def _construct_url(service, x, y, z):
    url = service.url()
    tile_url = url.format(x=x, y=y, z=z)
    return tile_url


def get_tile(lon, lat, zoom):
    return mt.tile(lon, lat, zoom)


def create_tile_xyz(x, y, z):
    return mt.Tile(x=x, y=y, z=z)


def get_tile_grid(w, s, e, n, zoom):
    return mt.tiles(w, s, e, n, [zoom])


def get_tile_bounds(tile):
    return mt.bounds(tile)


def get_tile_img(tile, service):
    x, y, z = tile.x, tile.y, tile.z

    url = _construct_url(service, x, y, z)
    image = _fetch_tile(url, 0.1, 2)
    return image