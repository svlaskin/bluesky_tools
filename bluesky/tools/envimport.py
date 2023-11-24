import json
import csv
import pandas as pd
import numpy as np
import requests
from geopy.geocoders import Nominatim
import urllib.parse

"""
Using Overpass API to gather points of interest within a specified area. These include:
1. Hospitals (for emergency flights)
2. Houses (for delivery locations)
3. Tall buildings and airports (for geofencing)
Currently, parcel centers are difficult to extract.
The input is a bounding box with a bottom left and top right point, or a city name.
"""
# Define the Overpass API query for the Amsterdam area
overpass_url = "http://overpass-api.de/api/interpreter"


# main function for autoimport.
def envimport(place_info, height=50):
    hospitals, geofences = [], []
    type_check = type(place_info)
    if type_check == str:
        print(f"City name specified is {place_info}")
        place_info = get_overpass_city_name(place_info).encode("utf-8")
        print(f'Corrected to {place_info.decode("latin-1")} for Overpass')
        bbox = get_city_bounding_box(place_info.decode("utf-8"))
        # run for name, loop through locations
        hospitals, geofences, del_locs = import_from_bbox(bbox, height)

    elif type_check == list or type_check == tuple:
        if len(place_info) == 4:
            print("Bounding box dimensions are correct.")
            # do the run again
            hospitals, geofences, del_locs = import_from_bbox(place_info, height)

        elif np.shape(place_info) == 4:
            print("Tuples in (lat_min,lon_min), (lat_max,lon_max) provided")

            # run the query for this too.
    else:
        print("Incorrect input type. Please provide either:")
        print("1. Bounding box in the form [lat_min,lon_min,lat_max,lon_max]")
        print("2. City Name as a String")

    return hospitals, geofences, del_locs


# --------------------- Main Functions --------------------------------------------------
# this is called if the input type is a city name string
def import_from_cityname(cityname, height):
    hospitals = amenity_from_name(cityname, "hospital")
    del_locs = del_loc_from_name(cityname)
    height_geofences = height_geofence_from_name(cityname, height)
    geofences = aero_geofence_from_name(cityname) # airports
    geofences.extend(height_geofences)
    return hospitals, geofences, del_locs


# this gets called if a bbox is given
def import_from_bbox(bbox, height):
    hospitals = amenity_from_bbox(bbox, "hospital")
    del_locs = del_loc_from_bbox(bbox)
    height_geofences = height_geofence_from_bbox(bbox, height)
    geofences = aero_geofence_from_bbox(bbox)
    geofences.extend(height_geofences)
    return hospitals, geofences, del_locs


# ---------------------- Helper functions ---------------------------------------------
def amenity_from_name(cityname, amenity_type):
    overpass_url = "https://overpass-api.de/api/interpreter"
    query = f"""
    [out:json];
    area[name="{cityname}"]->.a;
    (
    node(area.a)[amenity={amenity_type}];
    way(area.a)[amenity={amenity_type}];
    );
    out body;
    >;
    out skel qt;
    """

    # Send POST request to Overpass API
    response = requests.post(overpass_url, data=query)

    # Check if the request was successful
    if response.status_code == 200:
        # Load JSON data
        data = json.loads(response.text)

        amenity_data = []
        for element in data["elements"]:
            if "tags" in element and element["tags"].get("amenity") == str(
                amenity_type
            ):
                if "type" in element and element["type"] == "node":
                    amenity = {
                        "name": element["tags"].get("name", ""),
                        "latitude": element.get("lat", ""),
                        "longitude": element.get("lon", ""),
                    }
                elif "type" in element and element["type"] == "way":
                    # Calculate the average coordinates of the way
                    lats = []
                    lons = []
                    for node_id in element["nodes"]:
                        node = next(
                            (n for n in data["elements"] if n["id"] == node_id), None
                        )
                        if node:
                            lats.append(node.get("lat", 0))
                            lons.append(node.get("lon", 0))
                    if lats and lons:
                        avg_lat = sum(lats) / len(lats)
                        avg_lon = sum(lons) / len(lons)
                        amenity = {
                            "name": element["tags"].get("name", ""),
                            "latitude": avg_lat,
                            "longitude": avg_lon,
                        }
                    else:
                        continue
                else:
                    continue
                amenity_data.append(amenity)
        return amenity_data
    else:
        # Request was not successful, handle the error
        print(("Error: Request failed with status code", response.status_code))
        return []


# def amenity_from_name(cityname, amenity_type):
#     query = f"""
#     [out:json];
#     area[name="{cityname}"]->.a;
#     (
#     node(area.a)[amenity={amenity_type}];
#     way(area.a)[amenity={amenity_type}];
#     );
#     out body;
#     >;

#     out skel qt;
#     """

#     # Send POST request to Overpass API
#     response = requests.post(overpass_url, data=query)
#     print(response)
#     # Load JSON data
#     data = json.loads(response.text)

#     amenity_data = []
#     for element in data["elements"]:
#         if "tags" in element and element["tags"].get("amenity") == str(amenity_type):
#             if "type" in element and element["type"] == "node":
#                 amenity = {
#                     "name": element["tags"].get("name", ""),
#                     "latitude": element.get("lat", ""),
#                     "longitude": element.get("lon", ""),
#                 }
#             elif "type" in element and element["type"] == "way":
#                 # Calculate the average coordinates of the way
#                 lats = []
#                 lons = []
#                 for node_id in element["nodes"]:
#                     node = next(
#                         (n for n in data["elements"] if n["id"] == node_id), None
#                     )
#                     if node:
#                         lats.append(node.get("lat", 0))
#                         lons.append(node.get("lon", 0))
#                 if lats and lons:
#                     avg_lat = sum(lats) / len(lats)
#                     avg_lon = sum(lons) / len(lons)
#                     amenity = {
#                         "name": element["tags"].get("name", ""),
#                         "latitude": avg_lat,
#                         "longitude": avg_lon,
#                     }
#                 else:
#                     continue
#             else:
#                 continue
#             amenity_data.append(amenity)
#     return amenity_data


def amenity_from_bbox(bbox, amenity_type):
    query = f"""
    [out:json];
    (
    node[amenity={amenity_type}]({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]});
    way[amenity={amenity_type}]({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]});
    );
    out body;
    >;

    out skel qt;
    """

    # Send POST request to Overpass API
    response = requests.post(overpass_url, data=query)
    print(response)
    # Load JSON data
    data = json.loads(response.text)

    amenity_data = []
    for element in data["elements"]:
        if "tags" in element and element["tags"].get("amenity") == str(amenity_type):
            if "type" in element and element["type"] == "node":
                amenity = {
                    "name": element["tags"].get("name", ""),
                    "latitude": element.get("lat", ""),
                    "longitude": element.get("lon", ""),
                }
            elif "type" in element and element["type"] == "way":
                # Calculate the average coordinates of the way
                lats = []
                lons = []
                for node_id in element["nodes"]:
                    node = next(
                        (n for n in data["elements"] if n["id"] == node_id), None
                    )
                    if node:
                        lats.append(node.get("lat", 0))
                        lons.append(node.get("lon", 0))
                if lats and lons:
                    avg_lat = sum(lats) / len(lats)
                    avg_lon = sum(lons) / len(lons)
                    amenity = {
                        "name": element["tags"].get("name", ""),
                        "latitude": avg_lat,
                        "longitude": avg_lon,
                    }
                else:
                    continue
            else:
                continue
            amenity_data.append(amenity)
    clean_list = []

    for item in amenity_data:
        clean_list.append([item["name"], item["latitude"], item["longitude"]])
    return clean_list


def del_loc_from_name(cityname):
    # Convert bounding coordinates to string format
    # bbox_str = ",".join(str(coord) for coord in bbox)

    # Construct the Overpass query using f-string
    overpass_query = f"""
    [out:json];
    area[name="{cityname.decode('utf-8')}"]->.searchArea;
    (
    node["building"="house"](area.searchArea);
    way["building"="house"](area.searchArea);
    relation["building"="house"](area.searchArea);
    );
    out center;
    """

    # Send the query to the Overpass API
    response = requests.get(overpass_url, params={"data": overpass_query})

    # Check if the request was successful
    if response.status_code == 200:
        data = response.json()
        house_locations = []

        # Extract the coordinates of each house location
        for element in data["elements"]:
            if "center" in element:
                lat = element["center"]["lat"]
                lon = element["center"]["lon"]
                house_locations.append((lat, lon))

        return house_locations
    else:
        print("Error occurred while fetching house locations.")
        return []


def del_loc_from_bbox(bbox):
    # Convert bounding coordinates to string format
    bbox_str = ",".join(str(coord) for coord in bbox)

    # Construct the Overpass query using f-string
    overpass_query = f"""
    [out:json];
    (
    // Query for houses in the specified bounding box
    node["building"="house"]({bbox_str});
    way["building"="house"]({bbox_str});
    relation["building"="house"]({bbox_str});
    );
    out center;
    """
    # Send the query to the Overpass API
    response = requests.get(overpass_url, params={"data": overpass_query})

    # Check if the request was successful
    if response.status_code == 200:
        data = response.json()
        house_locations = []

        # Extract the coordinates of each house location
        for element in data["elements"]:
            if "center" in element:
                lat = element["center"]["lat"]
                lon = element["center"]["lon"]
                house_locations.append((lat, lon))

        return house_locations
    else:
        print("Error occurred while fetching house locations.")
        return []


def height_geofence_from_name(cityname, height):
    overpass_query = f"""
    [out:json][timeout:25];
    area[name="{cityname.decode('utf-8')}"]->.searchArea;
    (
    way[building][~"height"~"."](if:t["height"] > {height})(area.searchArea);
    );
    /*added by auto repair*/
    (._;>;);
    /*end of auto repair*/
    out body;
    """

    # Send the query to the Overpass API
    response = requests.get(overpass_url, params={"data": overpass_query})
    data = response.json()

    return extract_perimeters(data)


def height_geofence_from_bbox(bbox, height):
    print((bbox[0]))
    print((float(height)))
    print((bbox[0], bbox[1], bbox[2], bbox[3]))
    overpass_query = f"""
    [out:json][timeout:25];
    (
    way[building][~"height"~"."](if:t["height"] > {height})({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]});
    );
    /*added by auto repair*/
    (._;>;);
    /*end of auto repair*/
    out body;
    """

    # Send the query to the Overpass API
    response = requests.get(overpass_url, params={"data": overpass_query})
    data = response.json()

    return extract_perimeters(data)


def aero_geofence_from_name(cityname):
    overpass_query = f"""
    [out:json];
    area["name"="{cityname.decode('utf-8')}"]->.city;
    (
    way(area.city)["aeroway"="aerodrome"];
    relation(area.city)["aeroway"="aerodrome"];
    );
    out body;
    >;
    out skel qt;
    """
    response = requests.get(overpass_url, params={"data": overpass_query})
    data = response.json()

    return extract_perimeters(data, "AERO")


def aero_geofence_from_bbox(bbox):
    lat_min, lon_min, lat_max, lon_max = bbox

    overpass_query = f"""
    [out:json];
    (
    way["aeroway"="aerodrome"]({lat_min},{lon_min},{lat_max},{lon_max});
    relation["aeroway"="aerodrome"]({lat_min},{lon_min},{lat_max},{lon_max});
    );
    out body;
    >;
    out skel qt;
    """

    response = requests.get(overpass_url, params={"data": overpass_query})
    data = response.json()

    return extract_perimeters(data, "AERO")


# -------------------- Other tools ------------------------------------------------------------
def write_to_csv(hospitals, geofences, del_locs, folder="Sasha Experiments"):
    filenames = ["hospitals", "geofences", "del_locs"]
    data_to_write = [hospitals, geofences, del_locs]
    for i in range(len(filenames)):
        # file_path = f"bluesky-master/bluesky/{folder}/AAA{filenames[i]}.csv"
        file_path = f"AAA{filenames[i]}.csv"
        with open(file_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(data_to_write[i])

    return


# currently not working.
def filter_locations(locations):
    # broken at the moment, so just leave them in
    filtered_locations = []

    for item in locations:
        try:
            name = item[0]
            latitude = float(item[1])
            longitude = float(item[2])
            filtered_locations.append([name, latitude, longitude])
        except (IndexError, ValueError) as e:
            print(f"Error processing item {item}: {e}")

    # Extract latitude and longitude columns
    latitudes = [location[1] for location in filtered_locations]
    longitudes = [location[2] for location in filtered_locations]

    # Calculate average latitude and longitude
    avg_latitude = sum(latitudes) / len(latitudes)
    avg_longitude = sum(longitudes) / len(longitudes)

    # Filter locations based on deviation
    filtered_locations = [
        location
        for location in filtered_locations
        if abs(location[1] - avg_latitude) <= 2
        and abs(location[2] - avg_longitude) <= 2
    ]

    return filtered_locations


# def extract_perimeters(data,prefix='GEOF'):
#         # Process the response and extract perimeters
#     perimeters = []
#     unnamed_index = 1

#     for element in data["elements"]:
#         if element["type"] == "way":
#             name = element.get("tags", {}).get(
#                 "name", f"{prefix}{unnamed_index}"
#             )
#             if not element.get("tags", {}).get("name"):
#                 unnamed_index += 1

#             perimeter = []
#             for node_id in element["nodes"]:
#                 node = next(
#                     (
#                         n
#                         for n in data["elements"]
#                         if n["type"] == "node" and n["id"] == node_id
#                     ),
#                     None,
#                 )
#                 if node:
#                     lat = node.get("lat")
#                     lon = node.get("lon")
#                     if lat is not None and lon is not None:
#                         perimeter.append(f"{lat} {lon}")

#             if perimeter:
#                 perimeters.append(
#                     (name.replace(" ", ""), " ".join(perimeter))
#                 )

#     return perimeters


def extract_perimeters(data, prefix="GEOF"):
    perimeters = []
    unnamed_index = 1

    for element in data["elements"]:
        if element["type"] == "way":
            name = f"{prefix}{unnamed_index}"
            unnamed_index += 1

            perimeter = []
            for node_id in element["nodes"]:
                node = next(
                    (
                        n
                        for n in data["elements"]
                        if n["type"] == "node" and n["id"] == node_id
                    ),
                    None,
                )
                if node:
                    lat = node.get("lat")
                    lon = node.get("lon")
                    if lat is not None and lon is not None:
                        perimeter.append(f"{lat} {lon}")

            if perimeter:
                perimeters.append((name, " ".join(perimeter)))

    return perimeters


def get_city_center(city_name):
    geolocator = Nominatim(user_agent="city_center_finder")
    location = geolocator.geocode(city_name, exactly_one=True)

    if location:
        center = (location.latitude, location.longitude)
        return center
    else:
        return None


def get_city_bounding_box(city_name):
    geolocator = Nominatim(user_agent="city_bounding_box_finder")
    location = geolocator.geocode(city_name, exactly_one=True)

    if location:
        bounding_box = (
            location.raw["boundingbox"][0],
            location.raw["boundingbox"][2],
            location.raw["boundingbox"][1],
            location.raw["boundingbox"][3],
        )
        return bounding_box
    else:
        return None


def get_overpass_city_name(city_name):
    base_url = "https://nominatim.openstreetmap.org/search"
    params = {"q": city_name, "format": "json", "limit": 1}
    url = base_url + "?" + urllib.parse.urlencode(params)

    response = requests.get(url)
    if response.status_code == 200:
        data = response.json()
        if data and len(data) > 0:
            display_name = data[0].get("display_name")
            if display_name:
                return display_name.split(",")[0].strip()

    return None
