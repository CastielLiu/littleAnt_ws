#! /usr/bin/env python
# coding=utf-8
import math

a = 6378245.0
ee = 0.00669342162296594323

     
def wgs_gcj(lon, lat):

    dLat = transform_lat(lon - 105.0, lat - 35.0)
    dLon = transform_lon(lon - 105.0, lat - 35.0)
    radLat = lat / 180.0 * math.pi
    magic = math.sin(radLat)
    magic = 1 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * math.pi)
    dLon = (dLon * 180.0) / (a / sqrtMagic * math.cos(radLat) * math.pi)
    mgLat = lat + dLat
    mgLon = lon + dLon
    return mgLon, mgLat


def transform_lat(x, y):

    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def transform_lon(x, y):

    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

A = (118.81008229999999, 31.888423399999997)

print(wgs_gcj(A[0], A[1]))


def deg2rad(deg):
    return deg/180.0*math.pi


def convert(x1, y1):
    p1 = pyproj.Proj(proj = 'merc', datum = 'WGS84')
    p2 = pyproj.Proj(proj = 'latlon', datum = 'WGS84')

    x2, y2 = pyproj.transform(p1, p2, x1, y1)
    return x2, y2


from pyproj import Transformer
import pyproj

wgs84_utm = Transformer.from_crs(4326, 32750) # WGS84转UTM
utm_wgs84 = Transformer.from_crs(32750, 4326) # UTM转WGS84
x,y = wgs84_utm.transform(36.64918343, 117.02233187)
print(x,y, utm_wgs84.transform(x,y))




latitude = 36.64918343
longitude = 117.02233187
zone = int((longitude + 180.0) // 6.0) + 1
e, n = pyproj.Proj(proj='utm', datum='WGS84', zone=zone)(longitude, latitude)
print(e, n)

print(wgs84_utm.transform(latitude, longitude))

    

print(convert(e, n))

latlon1 = (33.376913888888886, 120.16214166666667)
latlon2 = (33.3794, 120.16574444444444)


print(wgs_gcj(120.16214166666667, 33.376913888888886))

























