<?xml version="1.0" encoding="UTF-8"?>
<!-- Synthetic CC0 fixture. This is not surveyed PLATEAU data. -->
<core:CityModel
    xmlns:core="http://www.opengis.net/citygml/2.0"
    xmlns:gml="http://www.opengis.net/gml"
    xmlns:bldg="http://www.opengis.net/citygml/building/2.0">
  <gml:boundedBy>
    <gml:Envelope srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
      <gml:lowerCorner>35.681201187 139.76693425 5</gml:lowerCorner>
      <gml:upperCorner>35.681398813 139.76726575 20</gml:upperCorner>
    </gml:Envelope>
  </gml:boundedBy>

  <!-- Deliberately reverse source order to exercise stable gml:id sorting. -->
  <core:cityObjectMember>
    <bldg:Building gml:id="bldg-B">
      <gml:name>South Annex</gml:name>
      <bldg:function>402</bldg:function>
      <bldg:measuredHeight uom="m">9</bldg:measuredHeight>
      <bldg:lod1Solid>
        <gml:Solid>
          <gml:exterior>
            <gml:CompositeSurface>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681326949 139.76713315 5 35.681326949 139.76726575 5 35.681398813 139.76726575 5 35.681398813 139.76713315 5 35.681326949 139.76713315 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681326949 139.76713315 14 35.681398813 139.76713315 14 35.681398813 139.76726575 14 35.681326949 139.76726575 14 35.681326949 139.76713315 14</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681326949 139.76713315 5 35.681326949 139.76713315 14 35.681326949 139.76726575 14 35.681326949 139.76726575 5 35.681326949 139.76713315 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681326949 139.76726575 5 35.681326949 139.76726575 14 35.681398813 139.76726575 14 35.681398813 139.76726575 5 35.681326949 139.76726575 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681398813 139.76726575 5 35.681398813 139.76726575 14 35.681398813 139.76713315 14 35.681398813 139.76713315 5 35.681398813 139.76726575 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681398813 139.76713315 5 35.681398813 139.76713315 14 35.681326949 139.76713315 14 35.681326949 139.76713315 5 35.681398813 139.76713315 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
            </gml:CompositeSurface>
          </gml:exterior>
        </gml:Solid>
      </bldg:lod1Solid>
    </bldg:Building>
  </core:cityObjectMember>

  <core:cityObjectMember>
    <bldg:Building gml:id="bldg-A">
      <gml:name>RNE City Hall</gml:name>
      <bldg:function>401</bldg:function>
      <bldg:measuredHeight uom="m">12</bldg:measuredHeight>
      <bldg:lod1Solid>
        <gml:Solid>
          <gml:exterior>
            <gml:CompositeSurface>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681201187 139.76693425 5 35.681201187 139.76706685 5 35.681291017 139.76706685 5 35.681291017 139.76693425 5 35.681201187 139.76693425 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681201187 139.76693425 17 35.681291017 139.76693425 17 35.681291017 139.76706685 17 35.681201187 139.76706685 17 35.681201187 139.76693425 17</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681201187 139.76693425 5 35.681201187 139.76693425 17 35.681201187 139.76706685 17 35.681201187 139.76706685 5 35.681201187 139.76693425 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681201187 139.76706685 5 35.681201187 139.76706685 17 35.681291017 139.76706685 17 35.681291017 139.76706685 5 35.681201187 139.76706685 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681291017 139.76706685 5 35.681291017 139.76706685 17 35.681291017 139.76693425 17 35.681291017 139.76693425 5 35.681291017 139.76706685 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
              <gml:surfaceMember><gml:Polygon><gml:exterior><gml:LinearRing><gml:posList srsDimension="3">35.681291017 139.76693425 5 35.681291017 139.76693425 17 35.681201187 139.76693425 17 35.681201187 139.76693425 5 35.681291017 139.76693425 5</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></gml:surfaceMember>
            </gml:CompositeSurface>
          </gml:exterior>
        </gml:Solid>
      </bldg:lod1Solid>
    </bldg:Building>
  </core:cityObjectMember>
</core:CityModel>
