import requests


class IPInfo:
    """
    Simple class for retrieving basic information about the currently
    used IP address (location, provider, timezone)
    """

    @staticmethod
    def get():
        try:
            return requests.get("http://ip-api.com/json").json()
        except Exception:
            # Something went wrong
            return None

    @staticmethod
    def get_coordinates():
        ip_info = IPInfo.get()
        if ip_info is not None and 'lat' in ip_info and 'lon' in ip_info:
            return '{},{}'.format(ip_info['lat'], ip_info['lon'])
        return None

    @staticmethod
    def get_location():
        ip_info = IPInfo.get()
        if ip_info is not None and 'city' in ip_info and 'country' in ip_info:
            return (ip_info['city'], ip_info['country'])
        return None
