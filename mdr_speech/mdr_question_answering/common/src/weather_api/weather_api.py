import requests
import urllib
import os
import re
import json


class WeatherApi(object):
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
                print("[WeatherApi] Location not found!")
                return None
        except Exception as e:
            # Something went wrong
            print("[WeatherApi] Could not retrieve location: " + str(e))
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
                print("[WeatherApi] Response didn't contain weather data for woeid " + str(woeid))
                return None
        except Exception as e:
            # Something went wrong
            print("[WeatherApi] Could not retrieve weather data: " + str(e))
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
        return (temperature - 32) * (5. / 9.)

    @staticmethod
    def condition_to_phrase(condition, temperature):
        # load all possible conditions from file
        cond_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../config/weather_conditions.json'))
        with open(cond_path, 'r') as cond_file:
            cond_data = json.loads(cond_file.read())
            replacements = cond_data['replacements']
            cond_adj = cond_data['conditions']['adjectives']
            cond_subs = cond_data['conditions']['substantives']
            cond_subs_sg = cond_data['conditions']['substantives_sg']
            sond_subs_pl = cond_data['conditions']['substantives_pl']

        # format condition: remove parentheses and whitespace, make lowercase
        condition = re.sub(r'\([^)]*\)', '', condition).strip().lower()
        if condition in replacements:
            condition = replacements[condition]

        # build a phrase
        if condition in cond_adj:
            phrase = "it's {} and {} degrees".format(condition, temperature)
        elif condition in cond_subs:
            phrase = "there is {} and it's {} degrees".format(condition, temperature)
        elif condition in cond_subs_sg:
            phrase = "there is a {} and it's {} degrees".format(condition, temperature)
        elif condition in cond_subs_pl:
            phrase = "there are {} and it's {} degrees".format(condition, temperature)
        else:
            phrase = "it's {} degrees".format(temperature)

        return phrase
