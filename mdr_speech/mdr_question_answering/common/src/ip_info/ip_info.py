import requests


class IPInfo(object):
    """
    Simple class for retrieving basic information about the currently
    used IP address (location, provider, timezone)
    """

    @staticmethod
    def get():
        try:
            return requests.get("http://ip-api.com/json").json()
        except Exception as e:
            # Something went wrong
            print("[IPInfo] Could not retrieve ip info: " + str(e))
            return None

    @staticmethod
    def get_coordinates():
        ip_info = IPInfo.get()
        if ip_info is not None and 'lat' in ip_info and 'lon' in ip_info:
            return '{},{}'.format(ip_info['lat'], ip_info['lon'])
        print("[IPInfo] Failed to get coordinates from ip info")
        return None

    @staticmethod
    def get_location():
        ip_info = IPInfo.get()
        if ip_info is not None and 'city' in ip_info and 'country' in ip_info:
            return (ip_info['city'], ip_info['country'])
        print("[IPInfo] Failed to get location from ip info")
        return None
