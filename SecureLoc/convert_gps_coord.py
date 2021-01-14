""" Converts coordinates in a Cartesian Reference System (CRS) into GPS coordinates (latitude + longitude)
Requires pyproj and utm libraries, install simply with: pip install pyproj utm"""
import pyproj
import utm
import math

# defining World Geodetic Format, should be set accordinly to the GPS device's WGS
WGS = "WGS84" 
r11_latlon = 43.649849877192985,1.3745697631578948
r11_cart = (-37.065, -6.410, -3.854) #  "x":-25.065, "y":-6.410, "z":-3.854,
r12_latlon = 43.64979264035088,1.374912307017544
r12_cart = (-17.510, 13.410, -3.854 )  #"x":-17.510, "y":13.410, "z":-3.854
# r13 = 43.64953705263158,1.3750360000000001  "x":11.660, "y":13.410, "z":-3.854

# std : 3E-6

class Point:
    """A representation of a 2D or 3D vector in a geolocation context"""
    def __init__(self, coords):
        if len(coords) == 2:
            self.x, self.y = coords
            self.z = None
        elif len(coords) == 3:
            self.x, self.y, self.z = coords
        else:
            raise ValueError("2D or 3D coordinates must be provded as an tuple or a list")
    
    def get_coord(self):
        """Returns the 2D or 3D coordinates as a tuple"""
        return((self.x, self.y, self.z) if self.z else (self.x, self.y))

class Vector:
    """A representation of a 2D or 3D vector in a geolocation context.
    Does not keep track of azimuth for 3D vectors.
    Can be initialized from both cartesian and polar coordinates, and from both 2D and 3D coordinates
    Attributes:
    x:      coordinate on the x axis
    y:      coordinate on the y axis
    z:      coordinate on the z axis (optional)
    arg:    polar angle
    norm:   polar radius
    """
    @staticmethod
    def to_polar(x, y, z = None):
        """Converts the provided 2D or 3D cartesian coordinates and returns the result.
        Note that for 3D coordinates the azimuth is NOT returned"""
        arg = math.atan2(y, x)
        norm = math.sqrt(y * y + x * x + z * z) if z else math.sqrt(y * y + x * x)
        return(norm, arg)

    @staticmethod
    def to_cartesian(norm, arg, z = None):
        """Converts polar coordinates to cartesian and returns the result.
        Does not handle azimuth - elevation is kept unchanged"""
        x = norm * math.cos(arg)
        y = norm * math.sin(arg)
        return(x,y, z)

    def __init__(self, x, y, z = None, norm = None, arg = None):
        self.x = x
        self.y = y
        self.z = z
        if norm and arg:
            self.arg = arg
            self.norm = norm
        else:
            self.norm, self.arg = Vector.to_polar(x, y, z)

    @classmethod 
    def from_polar(cls, norm, arg, z = None):
        """Enables an alternative constructor for polar coordinates"""
        (x, y, z) = Vector.to_cartesian(norm, arg, z)
        return(cls(x, y, z, norm, arg))

    @classmethod
    def from_point(cls, point):
        if type(point) is Point:
            return(cls(*point.get_coord()))
        else:
            raise TypeError("Two Points should be provided to build a vector from this constructor")
      
    @classmethod
    def from_points(cls, head, tail):
        if not(type(head) is Point and type(tail) is Point):
            raise TypeError("Two Points should be provided to build a vector from this constructor")
        elif (head.z and not(tail.z)) or (not(head.z) and (tail.z)):
            raise ValueError("Cannot create a vector from a 2D and a 3D points. Dimension should be consistent") 
        else:
            return(Vector.from_point(head).sub(Vector.from_point(tail)))
        
    def sum(self, vector_2sum):
        """Returns the sum of this vector with vector_2sum"""
        summed_coords = []
        for c1, c2 in zip(self.get_cartesian_coord(), vector_2sum.get_cartesian_coord()):
            summed_coords.append(c1 + c2)
        return(Vector(*summed_coords))

    def sub(self, vector_2sub):
        """Returns the difference of this vector with vector_2sub"""
        substracted_coords = []
        for c1, c2 in zip(self.get_cartesian_coord(), vector_2sub.get_cartesian_coord()):
            substracted_coords.append(c1 - c2)
        return(Vector(*substracted_coords))

    def get_angle_difference(self, vector):
        """Returns the difference of polar angles with the provided vector"""
        return(self.arg - vector.arg)
    
    def rotate_xy(self, angle):
        """Returns a copy of the vector rotated by the given angle on the xy plane"""
        return( Vector.from_polar(self.norm, self.arg + angle, self.z) )

    def get_cartesian_coord(self):
        """Returns the 2D or 3D coordinates as a tuple"""
        return((self.x, self.y, self.z) if self.z else (self.x, self.y))
    
    def get_polar_coord(self):
        """Returns the polar coordinates as a tuple"""
        return(self.norm, self.arg)

    def __str__(self):
        text = "{Cartesian: "
        for c in self.get_cartesian_coord():
            text += str(c) + ", "
        text += "Polar: " + str(self.norm) + ", " + str(self.arg) + "}"
        return(text)
   


def change_coordinates(p1, p2,coords):
    """Computes a coordinate system change.
    Parameters:
    - p1:       coordinates in the new sytem of the origin of the current system
    - p2:       coordinates in the new sytem of an arbitrary point on the x axis. Note: units are preserved (typically meters)
    - coords:   coordinates to convert in the new CRS
    Returns: converted coordinates"""
    
    x1, y1 = p1
    x2, y2 = p2
    
    # computing and normalizing absciss vector
    x_vect = [x2 - x1, y2 - y1]
    norm = x_vect[0] * x_vect[0] + x_vect[1] * x_vect[1]
    x_vect[0] /= norm
    x_vect[1] /= norm

    # defining an orthonormal sytem
    y_vect = (-x_vect[1], x_vect[0])

    # unpacking and converting provided coordinates
    x , y = coords
    new_x = x1 + x * x_vect[0] + y * y_vect[0]
    new_y = y1 + x * x_vect[1] + y * y_vect[1]
    return(new_x, new_y)


def convert_coordinates_to_gps(coords, p0_latlon, p1_latlon, p0_coord = (0,0), p1_coord = (1,0) , wgs = WGS):
    """Converts the coordinates provided in a given CRS to a latitude, longitude tuple
    @param p0_latlon:  (latitude, longitude) tuple of the beacon B0 with coordinates (0,0) in the CRS 
    @param p1_latlon: (latitude, longitude) tuple of the beacon B1; the axis (B0, B1) defines the X axis of the CRS. 
    Y axis is defined as the 2D axis orthonormal to (B0, B1)
    @param coords: The cartesian coordinates to convert to gps coordinates
    @return: a (latitude, longitude) tuple
    """
    try:
        # unpacking tuples
        (lat0, lon0) = p0_latlon
        (lat1, lon1) = p1_latlon     
    except:
        raise ValueError("Reference coordinates are unproperly formatted. (Latitude, Longitude) tuples are expected")

    # selecting UTM zone
    zone = utm.from_latlon(lat0, lon0)[2]

    # selecting hemisphere
    hemisphere = 'north' if lat0 > 0 else 'south'

    # forming the pyproj string defining the CRS
    # Setting projection to Univeral Transverse Mercator
    projstring = "+proj=utm"

    # defining UTM zone
    projstring += " +zone=" + str(zone)

    # defining hemisphère
    projstring += " +" + hemisphere

    # defining the world geodic system
    projstring += " +ellps=" + WGS
    projstring += " +datum=" + WGS

    # using meters as unit
    projstring += " +units=m"
    projstring += " +no_defs"

    # Applying UTM projection   
    p = pyproj.Proj(projstring, preserve_units=True)
    p0_utm = Point(p(lon0, lat0))
    p1_utm = Point(p(lon1, lat1))

    # point to convert is given in a CRS (Origin, vx, vy)
    # converting its coordinates in the CRS (p0, vs, vy) with a translation (p0,Origin )
    point_to_convert = Vector.from_point(Point(coords))
    local_translation = Vector.from_point(Point(p0_coord))
    point_to_convert = point_to_convert.sub(local_translation)

    # calculating the rotation angle
    utm_vect = Vector.from_points(p1_utm, p0_utm)
    cartesian_vect = Vector.from_points(Point(p1_coord), Point(p0_coord))
    rotation_angle = utm_vect.get_angle_difference(cartesian_vect)

    # translation to convert UTM CRS centered on p0 to a standard UTM CRS
    crs_translation = Vector.from_point(p0_utm)

    # performing the coordinate change with a (xy) rotation + origin translation
    utm_coords = point_to_convert.rotate_xy(rotation_angle).sum(crs_translation).get_cartesian_coord()
    utm_x, utm_y = utm_coords[:2]
    
    # reverse projection to convert UTM coords back to geodetic coordinates
    lon, lat = p(utm_x, utm_y , inverse=True)
    return(lat, lon)


if __name__ == "__main__":
    # sample test programm
    # test coordinates (points to Blagnac IUT)
    ANCHOR0_LATLON = 43.647926403525446, 1.3752879893023537
    ANCHOR1_LATLON = 43.64917628655427, 1.3754650150912193
    ANCHOR0 = 20, 0
    ANCHOR1=  100, 0
    # print(convert_coordinates_to_gps( (200, -50), ANCHOR0_LATLON, ANCHOR1_LATLON, (0, 0), (100, 0) ) )
    test_coord = (0, 0, 0)
    print(convert_coordinates_to_gps(test_coord, r11_latlon, r12_latlon, r11_cart, r12_cart))
    # v1 = Vector(1, 2)
    # v2 = Vector(3, 4)
    # v7= v2.rotate_xy(1.5)
    # v8= v7.sum(v1)
    # p1 = Point((1, 2))
    # p2 = Point((5, 6))
    # v5 = Vector.from_points(p2, p1)
    # v6 = v5.sub(v2)
    # # v2 = v.rotate_xy(0.3)
    # print(v2, v7, v8)
