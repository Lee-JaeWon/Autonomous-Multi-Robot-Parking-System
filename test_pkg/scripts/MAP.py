#!/usr/bin/env python3


class MapData():
    
    def __init__(self,map,resolution,height,width,origin_x,origin_y):
        self.__map = map
        self.__resolution = resolution
        self.__height = height
        self.__width = width
        self.__frame_id = "map"
        self.__origin_x = origin_x
        self.__origin_y = origin_y
    
    def getOriginX(self): return self.__origin_x
    def getOriginY(self): return self.__origin_y
    def getResolution(self): return self.__resolution
    def getCellSizeX(self): return self.__width
    def getCellSizeY(self): return self.__height
    def getSizeInMeterX(self): return (self.getCellSizeX() - 1 + 0.5) * self.getResolution()
    def getSizeInMeterY(self): return (self.getCellSizeY() - 1 + 0.5) * self.getResolution()
    def getMap(self): return self.__map
    def getFrameId(self): return self.__frame_id
    
    
