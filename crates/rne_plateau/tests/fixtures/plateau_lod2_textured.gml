<?xml version="1.0" encoding="UTF-8"?>
<!-- Synthetic CC0 fixture. This is not surveyed PLATEAU data. -->
<core:CityModel
    xmlns:core="http://www.opengis.net/citygml/2.0"
    xmlns:gml="http://www.opengis.net/gml"
    xmlns:bldg="http://www.opengis.net/citygml/building/2.0"
    xmlns:app="http://www.opengis.net/citygml/appearance/2.0">
  <gml:boundedBy>
    <gml:Envelope srsName="urn:ogc:def:crs:EPSG::6677" srsDimension="3">
      <gml:lowerCorner>0 0 0</gml:lowerCorner>
      <gml:upperCorner>10 10 10</gml:upperCorner>
    </gml:Envelope>
  </gml:boundedBy>
  <core:cityObjectMember>
    <bldg:Building gml:id="lod2-building">
      <gml:name>Textured LOD2 Building</gml:name>
      <bldg:boundedBy>
        <bldg:WallSurface gml:id="wall-surface">
          <bldg:lod2MultiSurface>
            <gml:MultiSurface>
              <gml:surfaceMember>
                <gml:Polygon gml:id="wall-polygon">
                  <gml:exterior>
                    <gml:LinearRing gml:id="wall-ring">
                      <gml:posList srsDimension="3">0 0 0 10 0 0 10 0 10 0 0 10 0 0 0</gml:posList>
                    </gml:LinearRing>
                  </gml:exterior>
                </gml:Polygon>
              </gml:surfaceMember>
            </gml:MultiSurface>
          </bldg:lod2MultiSurface>
        </bldg:WallSurface>
      </bldg:boundedBy>
      <bldg:boundedBy>
        <bldg:RoofSurface gml:id="roof-surface">
          <bldg:lod2MultiSurface>
            <gml:MultiSurface>
              <gml:surfaceMember>
                <gml:Polygon gml:id="roof-polygon">
                  <gml:exterior>
                    <gml:LinearRing gml:id="roof-ring">
                      <gml:posList srsDimension="3">0 0 10 10 0 10 10 10 10 0 10 10 0 0 10</gml:posList>
                    </gml:LinearRing>
                  </gml:exterior>
                </gml:Polygon>
              </gml:surfaceMember>
            </gml:MultiSurface>
          </bldg:lod2MultiSurface>
        </bldg:RoofSurface>
      </bldg:boundedBy>
      <bldg:boundedBy>
        <bldg:GroundSurface gml:id="ground-surface">
          <bldg:lod2MultiSurface>
            <gml:MultiSurface>
              <gml:surfaceMember>
                <gml:Polygon gml:id="ground-polygon">
                  <gml:exterior>
                    <gml:LinearRing gml:id="ground-ring">
                      <gml:posList srsDimension="3">0 0 0 0 10 0 10 10 0 10 0 0 0 0 0</gml:posList>
                    </gml:LinearRing>
                  </gml:exterior>
                </gml:Polygon>
              </gml:surfaceMember>
            </gml:MultiSurface>
          </bldg:lod2MultiSurface>
        </bldg:GroundSurface>
      </bldg:boundedBy>
    </bldg:Building>
  </core:cityObjectMember>
  <app:appearanceMember>
    <app:Appearance>
      <app:theme>rgbTexture</app:theme>
      <app:surfaceDataMember>
        <app:ParameterizedTexture>
          <app:imageURI>appearance/facade.png</app:imageURI>
          <app:mimeType>image/png</app:mimeType>
          <app:target uri="#wall-polygon">
            <app:TexCoordList>
              <app:textureCoordinates ring="#wall-ring">0 0 1 0 1 1 0 1 0 0</app:textureCoordinates>
            </app:TexCoordList>
          </app:target>
        </app:ParameterizedTexture>
      </app:surfaceDataMember>
    </app:Appearance>
  </app:appearanceMember>
</core:CityModel>
