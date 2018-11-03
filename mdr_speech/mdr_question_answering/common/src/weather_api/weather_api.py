import requests
import urllib


class WeatherApi:
    """
    Provides functions for retrieving location and weather information from
    the yahoo APIs
    """
    YAHOO_API_URL = "https://query.yahooapis.com/v1/public/yql?q={}&format=json"

    @staticmethod
    def build_url(query):
        """
        Builds the full API url from a query string
        """
        return WeatherApi.YAHOO_API_URL.format(urllib.quote_plus(query))

    @staticmethod
    def find_location(searchstring):
        """
        Retrieves location info for a given search string using some undocumented(?) yahoo maps api
        :param searchstring: Any search string, e.g. city name, address, zip code, coordinates, ...
        :returns: A dict containing the address details (e.g. 'country', 'city', 'woeid')
                  or None if no location could be found or an exception occured
        """
        query_string = "select * from xml where url = 'http://gws2.maps.yahoo.com/findlocation?pf=1&locale=en_US&offset=15&flags=&gflags=R&q={}'"
        request_url = WeatherApi.build_url(query_string.format(urllib.quote_plus(searchstring)))

        try:
            response = requests.get(request_url).json()
            if response['query']['count'] == 1 and int(response['query']['results']['ResultSet']['Found']) >= 1:
                result = response['query']['results']['ResultSet']['Result']
                return result[0] if isinstance(result, list) else result
            else:
                # Location not found
                return None
        except Exception:
            # Something went wrong
            return None

    @staticmethod
    def get_weather_by_woeid(woeid):
        """
        Requests current weather conditions for a given location from yahoo weather api
        :param location: either a city name, zip code or latitude/longitue
        :returns: If weather data could be retrieved, a tuple containing the
                  short description (like "sunny") and the temperature in
                  celsius is returned. Example: ("sunny", 24)
                  If the location could not be found or an error occured,
                  None is returned
        """
        query_string = "select item.title, item.condition from weather.forecast where woeid={} and u='c'"
        request_url = WeatherApi.build_url(query_string.format(woeid))

        try:
            result = requests.get(request_url).json()
            if result['query']['count'] == 1:
                weather_data = result['query']['results']['channel']['item']
                return (weather_data['condition']['text'], int(weather_data['condition']['temp']))
            else:
                # No weather data for woeid?
                return None
        except Exception:
            # Something went wrong
            return None

    @staticmethod
    def get_weather(location):
        loc = WeatherApi.find_location(location)
        if loc is not None:
            return WeatherApi.get_weather_by_woeid(loc['woeid'])
        else:
            return None

    @staticmethod
    def fahrenheit_to_celsius(temperature):
        """
        Converts a given temperature from fahrenheit to celsius
        """
        return (temperature - 32) * (5/9)
