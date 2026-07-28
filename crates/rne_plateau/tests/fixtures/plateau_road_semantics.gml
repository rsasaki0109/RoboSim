<?xml version="1.0" encoding="UTF-8"?>
<!-- Synthetic CC0 fixture covering PLATEAU road LOD2 and LOD3.1 semantics. -->
<core:CityModel
    xmlns:core="http://www.opengis.net/citygml/2.0"
    xmlns:gml="http://www.opengis.net/gml"
    xmlns:tran="http://www.opengis.net/citygml/transportation/2.0">
  <gml:boundedBy>
    <gml:Envelope srsName="urn:ogc:def:crs:EPSG::6677" srsDimension="3">
      <gml:lowerCorner>0 -6 0</gml:lowerCorner>
      <gml:upperCorner>78 6 0</gml:upperCorner>
    </gml:Envelope>
  </gml:boundedBy>

  <core:cityObjectMember>
    <tran:Road gml:id="road-lod2">
      <gml:name>Semantic Avenue LOD2</gml:name>
      <tran:class>1</tran:class>
      <tran:function>1</tran:function>
      <tran:function>2</tran:function>
      <tran:trafficArea>
        <tran:TrafficArea gml:id="lod2-carriageway">
          <tran:class>1</tran:class>
          <tran:function>1000</tran:function>
          <tran:lod2MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">0 -4 0 30 -4 0 30 4 0 0 4 0 0 -4 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod2MultiSurface>
        </tran:TrafficArea>
      </tran:trafficArea>
      <tran:trafficArea>
        <tran:TrafficArea gml:id="lod2-sidewalk">
          <tran:function>2000</tran:function>
          <tran:lod2MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">0 4 0 30 4 0 30 6 0 0 6 0 0 4 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod2MultiSurface>
        </tran:TrafficArea>
      </tran:trafficArea>
      <tran:auxiliaryTrafficArea>
        <tran:AuxiliaryTrafficArea gml:id="lod2-island">
          <tran:function>3000</tran:function>
          <tran:lod2MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">12 -0.5 0 18 -0.5 0 18 0.5 0 12 0.5 0 12 -0.5 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod2MultiSurface>
        </tran:AuxiliaryTrafficArea>
      </tran:auxiliaryTrafficArea>
    </tran:Road>
  </core:cityObjectMember>

  <core:cityObjectMember>
    <tran:Road gml:id="road-lod31">
      <gml:name>Semantic Avenue LOD3.1</gml:name>
      <tran:class>1</tran:class>
      <tran:function>3</tran:function>
      <tran:trafficArea>
        <tran:TrafficArea gml:id="lod31-lane-a">
          <tran:function>1010</tran:function>
          <tran:lod3MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">40 -3 0 70 -3 0 70 0 0 40 0 0 40 -3 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod3MultiSurface>
        </tran:TrafficArea>
      </tran:trafficArea>
      <tran:trafficArea>
        <tran:TrafficArea gml:id="lod31-lane-b">
          <tran:function>1010</tran:function>
          <tran:lod3MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">40 0 0 70 0 0 70 3 0 40 3 0 40 0 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod3MultiSurface>
        </tran:TrafficArea>
      </tran:trafficArea>
      <tran:trafficArea>
        <tran:TrafficArea gml:id="lod31-roadway-remainder">
          <tran:function>1000</tran:function>
          <tran:lod3MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">40 3 0 70 3 0 70 4 0 40 4 0 40 3 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod3MultiSurface>
        </tran:TrafficArea>
      </tran:trafficArea>
      <tran:trafficArea>
        <tran:TrafficArea gml:id="lod31-intersection">
          <tran:function>1020</tran:function>
          <tran:lod3MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">70 -4 0 78 -4 0 78 4 0 70 4 0 70 -4 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod3MultiSurface>
        </tran:TrafficArea>
      </tran:trafficArea>
      <tran:auxiliaryTrafficArea>
        <tran:AuxiliaryTrafficArea gml:id="lod31-median">
          <tran:function>3000</tran:function>
          <tran:lod3MultiSurface>
            <gml:MultiSurface><gml:surfaceMember><gml:Polygon>
              <gml:exterior><gml:LinearRing><gml:posList srsDimension="3">50 -0.25 0 60 -0.25 0 60 0.25 0 50 0.25 0 50 -0.25 0</gml:posList></gml:LinearRing></gml:exterior>
            </gml:Polygon></gml:surfaceMember></gml:MultiSurface>
          </tran:lod3MultiSurface>
        </tran:AuxiliaryTrafficArea>
      </tran:auxiliaryTrafficArea>
    </tran:Road>
  </core:cityObjectMember>
</core:CityModel>
