function g = Zgravity_fn(n,e,alt)
ellipsoid = wgs84Ellipsoid;
[lat,lon,h] = ned2geodetic(n,e,alt,40.976111, 28.814167,0,ellipsoid);
g = gravity(lat,h);
end