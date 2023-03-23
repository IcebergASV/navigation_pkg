import math

def meters_per_degree():
    city = raw_input("St. John's (type 'SJ') or Sarasota (type 'SA') ?")
    cm = raw_input("how many centimeters?")

    cm = int(cm)

    #City cordinates
    if city == 'SJ':
        lat = 47.5556*(math.pi/180)
    elif city == 'SA':
        lat = 27.3365*(math.pi/180)
    else:
        print("Incorrect input -- exiting")
        return


    #meters per degree latitude
    meters_per_lat = 111132.92 - 559.82*math.cos(2*lat) + 1.175*math.cos(4*lat) - 0.0023*math.cos(6*lat)
    #meters per degree longitude
    meters_per_lon = 111412.84*math.cos(lat) - 93.5*math.cos(3*lat) + 0.118*math.cos(5*lat)

    #degrees per cm
    lat_per_cm = cm * (1 / (meters_per_lat * 100))
    lon_per_cm = cm * (1 / (meters_per_lon * 100))

    print("Meters per degree latitude = " + str(meters_per_lat))
    print("Meters per degree longitude = " + str(meters_per_lon))
    print(str(cm) + "cm : " + str(lat_per_cm) + " degrees latitude")
    print(str(cm) + "cm: " + str(lon_per_cm) + " degrees longitude")
    

    return

meters_per_degree()