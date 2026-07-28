<?xml version="1.0" encoding="UTF-8"?>
<core:CityModel xmlns:app="http://www.opengis.net/citygml/appearance/2.0" xmlns:bldg="http://www.opengis.net/citygml/building/2.0" xmlns:brid="http://www.opengis.net/citygml/bridge/2.0" xmlns:core="http://www.opengis.net/citygml/2.0" xmlns:dem="http://www.opengis.net/citygml/relief/2.0" xmlns:frn="http://www.opengis.net/citygml/cityfurniture/2.0" xmlns:gen="http://www.opengis.net/citygml/generics/2.0" xmlns:gml="http://www.opengis.net/gml" xmlns:grp="http://www.opengis.net/citygml/cityobjectgroup/2.0" xmlns:luse="http://www.opengis.net/citygml/landuse/2.0" xmlns:pbase="http://www.opengis.net/citygml/profiles/base/2.0" xmlns:sch="http://www.ascc.net/xml/schematron" xmlns:smil20="http://www.w3.org/2001/SMIL20/" xmlns:smil20lang="http://www.w3.org/2001/SMIL20/Language" xmlns:tex="http://www.opengis.net/citygml/texturedsurface/2.0" xmlns:tran="http://www.opengis.net/citygml/transportation/2.0" xmlns:tun="http://www.opengis.net/citygml/tunnel/2.0" xmlns:uro="https://www.geospatial.jp/iur/uro/3.1" xmlns:veg="http://www.opengis.net/citygml/vegetation/2.0" xmlns:wtr="http://www.opengis.net/citygml/waterbody/2.0" xmlns:xAL="urn:oasis:names:tc:ciq:xsdschema:xAL:2.0" xmlns:xlink="http://www.w3.org/1999/xlink" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://www.geospatial.jp/iur/uro/3.1 
../../schemas/iur/uro/3.1/urbanObject.xsd 
http://www.opengis.net/citygml/2.0 
http://schemas.opengis.net/citygml/2.0/cityGMLBase.xsd 
http://www.opengis.net/citygml/landuse/2.0 
http://schemas.opengis.net/citygml/landuse/2.0/landUse.xsd 
http://www.opengis.net/citygml/building/2.0 
http://schemas.opengis.net/citygml/building/2.0/building.xsd 
http://www.opengis.net/citygml/transportation/2.0 
http://schemas.opengis.net/citygml/transportation/2.0/transportation.xsd 
http://www.opengis.net/citygml/generics/2.0 
http://schemas.opengis.net/citygml/generics/2.0/generics.xsd 
http://www.opengis.net/citygml/cityobjectgroup/2.0 
http://schemas.opengis.net/citygml/cityobjectgroup/2.0/cityObjectGroup.xsd 
http://www.opengis.net/gml 
http://schemas.opengis.net/gml/3.1.1/base/gml.xsd">
	<gml:boundedBy>
		<gml:Envelope srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
			<gml:lowerCorner>37.63014082933306 138.95319543200188 0</gml:lowerCorner>
			<gml:upperCorner>37.63343684849707 138.957080680575 0</gml:upperCorner>
		</gml:Envelope>
	</gml:boundedBy>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_c2f90b31-c8dc-4bee-a88f-3c060d2e8293">
			<gml:name>市道1454号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63311007627029 138.95557828368408 0 37.63302395058377 138.95557401894402 0 37.633031240737616 138.95559955624793 0 37.633032035032286 138.95562720662076 0 37.63302749022429 138.95575407634837 0 37.63302039710098 138.95584082182953 0 37.632988785406184 138.95611651729382 0 37.6330574845035 138.95613008015462 0 37.63307055617333 138.95605855358028 0 37.63307798569656 138.9560013817815 0 37.633084122201986 138.95595235977208 0 37.63308781053674 138.95590728834708 0 37.63309936797685 138.9557612001155 0 37.63311007627029 138.95557828368408 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_bf7d608d-8193-4219-9463-d273a101049c">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63311007627029 138.95557828368408 0 37.633129299141935 138.9553604090042 0 37.633055909259184 138.95534817783155 0 37.63302395058377 138.95557401894402 0 37.63311007627029 138.95557828368408 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_6d514af4-7923-48ac-a46c-92c02a2f3b8a">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63258401231072 138.95544865380523 0 37.63258150877673 138.9554384162445 0 37.63251445631148 138.9556751450531 0 37.63255373109769 138.95563153754006 0 37.63262612807921 138.9556208760981 0 37.6327384646448 138.95560445353468 0 37.63286378416279 138.95558607076626 0 37.63299064683021 138.95556475152608 0 37.633023950583755 138.95557401894402 0 37.63305590925916 138.95534817783155 0 37.63304365416837 138.9553716697317 0 37.63303631195647 138.95538295506122 0 37.63302735966535 138.95539117142735 0 37.633020043689896 138.95539565883408 0 37.63299866962644 138.95540051365424 0 37.63294385227468 138.95540890331822 0 37.632893451803255 138.95541686672112 0 37.63278273365122 138.9554342058835 0 37.63276803870693 138.95543615561692 0 37.63267455067343 138.9554483880469 0 37.6326009867189 138.95545790953358 0 37.632589641394055 138.95545580087813 0 37.63258401231072 138.95544865380523 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_ffb2b834-4974-426b-b798-83f447eb9b91">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63251445631148 138.9556751450531 0 37.63258150877673 138.9554384162445 0 37.632406008367944 138.95545456682467 0 37.6323838224344 138.95562302232813 0 37.632389557636564 138.9556312149693 0 37.63251445631148 138.9556751450531 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_8879df86-2eea-4aab-94d7-bff8c20c4b32">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.632383822434385 138.95562302232815 0 37.632406008367916 138.9554545668247 0 37.63231243852172 138.95544130612504 0 37.63224770700163 138.95542697515296 0 37.63219547817314 138.9554183855078 0 37.632189773162416 138.95544905500495 0 37.63204734334375 138.95541816129347 0 37.63186695140044 138.95537094751404 0 37.63170933319883 138.95532976448098 0 37.63153957400573 138.95528250270556 0 37.631430660909224 138.9552523805489 0 37.63142514191931 138.95532825713593 0 37.631530819969576 138.9553560936807 0 37.63168285604515 138.95539610961055 0 37.631694918275436 138.95539924229828 0 37.6318396379154 138.95544385903636 0 37.631851697957316 138.955447558223 0 37.63203532183073 138.95549785099897 0 37.63218562966643 138.9555420490505 0 37.632227097525494 138.95556054350035 0 37.63226283454985 138.95556971237232 0 37.63225954657809 138.9556044754471 0 37.63229304673014 138.95560955188012 0 37.632383822434385 138.95562302232815 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_41b44c72-4c5d-465f-8d80-61132bda4882">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63142514191932 138.95532825713593 0 37.631430660909224 138.95525238054887 0 37.63138414013312 138.95523566843042 0 37.63137426945602 138.95531842964903 0 37.63142514191932 138.95532825713593 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_e9215a14-2e4d-4a3d-b78f-a6c67ca6366d">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63137426945602 138.95531842964903 0 37.63138414013311 138.95523566843042 0 37.63137649040813 138.95523324249177 0 37.6311920676896 138.95517977359597 0 37.63100603048289 138.95512436905287 0 37.630688841235425 138.9550342890375 0 37.63065288096035 138.95510635372392 0 37.63068095912643 138.95511547547147 0 37.630991581589555 138.95520268331165 0 37.631178428390164 138.95525843287302 0 37.631361322788976 138.95531098623826 0 37.63136213154579 138.95531155766648 0 37.63136860728401 138.95531465624288 0 37.63137426945602 138.95531842964903 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_714b9089-5786-4c79-8984-54aa8318253a">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63065288096035 138.95510635372392 0 37.63068884123542 138.9550342890375 0 37.63064052487881 138.9550158668128 0 37.63060501513882 138.955087934209 0 37.63065288096035 138.95510635372392 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_7ad03a1d-a99a-4960-8102-ff954b9beb20">
			<gml:name>主要地方道長岡・見附・三条線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">3</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.630605015138826 138.95508793420902 0 37.630640524878814 138.9550158668128 0 37.63061109469601 138.95500685016248 0 37.63015329881371 138.95488033882816 0 37.63014082933306 138.95495945738688 0 37.630297190618556 138.95499949810366 0 37.63050222778508 138.9550581899714 0 37.63059691575962 138.95508527897445 0 37.630605015138826 138.95508793420902 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_be48c1da-fdb6-4205-ac81-16bf2371fd55">
			<gml:name>市道1454号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.633247919828506 138.9533376785215 0 37.63327621705669 138.953219450145 0 37.63327769790168 138.95320926198505 0 37.63317002271513 138.95320860804364 0 37.63316908249883 138.95321879947235 0 37.633164580769325 138.95328822597332 0 37.633247919828506 138.9533376785215 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_1071897e-ff01-4862-b621-2fa332a83e09">
			<gml:name>市道1168号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63324791982851 138.95333767852148 0 37.63316458076936 138.95328822597332 0 37.63314998108548 138.95345299099253 0 37.63322551804524 138.95344585878718 0 37.63324791982851 138.95333767852148 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_0f1da862-0936-42d5-a6cf-0456b15d905a">
			<gml:name>市道1454号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63319334476764 138.9539158648379 0 37.63319640268236 138.95382365595052 0 37.63320908890223 138.95366454435117 0 37.63322224637289 138.95347654364687 0 37.633225518045215 138.95344585878718 0 37.63314998108544 138.95345299099256 0 37.63313730590805 138.95365617685118 0 37.6331245217514 138.95381732713 0 37.63312186305367 138.95389945452646 0 37.63319334476764 138.9539158648379 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_bc19967e-77bd-4a56-b62e-299cf3c39bc6">
			<gml:name>市道1454号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63319334476766 138.9539158648379 0 37.63312186305368 138.95389945452646 0 37.63311931446764 138.95404661762734 0 37.63319113053112 138.95409250247522 0 37.63319334476766 138.9539158648379 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_aa46ce0e-d5af-41cb-932c-d711b52a276e">
			<gml:name>市道1454号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.6331911305311 138.95409250247522 0 37.63311931446762 138.95404661762737 0 37.633118664951965 138.95412173254556 0 37.63311444058296 138.95432961501055 0 37.63310863848267 138.9545259310929 0 37.633103622162565 138.95461178302938 0 37.63310256247075 138.95462956489922 0 37.633095840351324 138.95473716030105 0 37.63309440974113 138.95475765914102 0 37.633074201052736 138.9550445284164 0 37.63306925122269 138.9551130455706 0 37.633058212480215 138.95526502868253 0 37.63305590925916 138.95534817783155 0 37.63312929914192 138.9553604090042 0 37.63313028009248 138.95526966050778 0 37.63314589296186 138.95505312329138 0 37.63316771773412 138.95474439639537 0 37.63317444248018 138.95463612109737 0 37.633180614277336 138.95453101485504 0 37.633186423804055 138.95433277250243 0 37.63319074438978 138.95412330420064 0 37.6331911305311 138.95409250247522 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_89dd7097-68ed-4d6a-bdaf-a4bf740a2f32">
			<gml:name>市道1455号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63259672785336 138.95693226505884 0 37.63260909891053 138.95685563624815 0 37.63261926854869 138.95681230434104 0 37.63262373017725 138.95680032177833 0 37.6326337447757 138.95677375741218 0 37.6326562569767 138.95673095422325 0 37.63268346995667 138.95668421424855 0 37.63275368150906 138.95660963873019 0 37.63283869605402 138.95652846887262 0 37.632888257555514 138.95648118561078 0 37.63283661320805 138.9564147019009 0 37.63280437262742 138.9564334259758 0 37.63265493878579 138.95658263547529 0 37.632626557553756 138.956628575216 0 37.6325985297064 138.95667632989498 0 37.63257066187481 138.95672929734596 0 37.632551723159814 138.95678005340085 0 37.632538989804765 138.95683390653252 0 37.63252605882851 138.95691551707105 0 37.63259672785336 138.95693226505884 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_96556b12-a0b7-4fb5-ad13-95c4bcd523b4">
			<gml:name>市道1168号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63314998108548 138.95345299099253 0 37.63316458076936 138.95328822597332 0 37.63315570740983 138.9532993889244 0 37.633143501479104 138.95331019172417 0 37.633129975318056 138.95331282879354 0 37.63311925111056 138.95331321685362 0 37.63307200618261 138.9532972942648 0 37.6330685264843 138.95328832231777 0 37.63306812253469 138.95327619661535 0 37.63304601069623 138.95340273322086 0 37.633050350527796 138.95339890734263 0 37.63305468165159 138.95339734743848 0 37.63305648200523 138.9533978115825 0 37.6331038357462 138.95340886293056 0 37.633108517275055 138.9534099110883 0 37.63311364715071 138.95341152847755 0 37.633131286348586 138.9534171874256 0 37.63314998108548 138.95345299099253 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_9526967d-a5cd-448e-9cff-f236eca8e9ca">
			<gml:name>市道1457号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63311931446764 138.95404661762734 0 37.63312186305368 138.95389945452646 0 37.63309545482958 138.9539246734183 0 37.633064311712715 138.95391587307478 0 37.63303218528835 138.9539050273337 0 37.63302192642259 138.95390156589397 0 37.63297539369652 138.95388791331112 0 37.63297170332427 138.95388687115394 0 37.63296892358828 138.95388334190838 0 37.63296822628593 138.95387721940048 0 37.63293961070609 138.9539843415844 0 37.63294928373843 138.95397612944328 0 37.63295513967552 138.9539763916703 0 37.63296045542001 138.9539765373099 0 37.63300689759491 138.95399030269553 0 37.633013736836986 138.95399261032952 0 37.63304496307758 138.95400322402622 0 37.6330977808867 138.95402269330606 0 37.63311931446764 138.95404661762734 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_23d69b10-c2f2-47c6-8b63-62aa71deaaf0">
			<gml:name>市道1168号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63306812253469 138.95327619661535 0 37.63303591769688 138.95326229153636 0 37.6330319935726 138.95327518405358 0 37.63301334490075 138.95339154452228 0 37.63301376581865 138.9533992515685 0 37.63304601069623 138.95340273322086 0 37.63306812253469 138.95327619661535 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_fbaddc41-dc99-4959-8504-5793b67f2223">
			<gml:name>市道1456号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63306812253469 138.95327619661535 0 37.63307726574837 138.9532182418338 0 37.633078836697884 138.95320805424822 0 37.63304441667915 138.95320784520752 0 37.63304284572966 138.95321803278844 0 37.63303591769688 138.95326229153636 0 37.63306812253469 138.95327619661535 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_7dca3426-4e53-41ba-9f83-91623802fd9d">
			<gml:name>市道1168号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63239196806564 138.95320388280444 0 37.63239187796087 138.9532038822572 0 37.63239225999957 138.95320396220998 0 37.63243031353047 138.95321431271572 0 37.632508706001666 138.95323790211825 0 37.632554966319304 138.9532521190562 0 37.63263641973127 138.95327640693586 0 37.63270842258327 138.95329791824278 0 37.63280779072451 138.95332650718754 0 37.63293542084223 138.95336331229052 0 37.63298501327508 138.9533780028395 0 37.633000853862825 138.95338274443165 0 37.63300948911717 138.95338664913987 0 37.633013344900725 138.9533915445223 0 37.63303199357258 138.9532751840536 0 37.633027374290016 138.95328138756855 0 37.63301240384153 138.95328469567414 0 37.632953090528865 138.9532672269074 0 37.63280158131225 138.95323061683345 0 37.6327263977995 138.953216110901 0 37.63269039506097 138.95320569517347 0 37.63239196806564 138.95320388280444 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_1c34e5cf-2dd5-43e6-ad0c-6c69e6746242">
			<gml:name>市道1158号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63304601069623 138.95340273322086 0 37.63301376581865 138.9533992515685 0 37.633002605376156 138.9534662581236 0 37.6329950425978 138.95351153267822 0 37.63298548573793 138.9535598542379 0 37.632974273867944 138.95361677661973 0 37.632940388442215 138.95373531029253 0 37.63293282347542 138.9537576979296 0 37.632904089369035 138.95384310133014 0 37.63296822628593 138.95387721940048 0 37.63298589953391 138.95375688765003 0 37.63300802438359 138.95362706559428 0 37.63302762568046 138.95352079480122 0 37.63304601069623 138.95340273322086 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_f47e79f1-678c-4f4b-9039-b553e009e418">
			<gml:name>市道1158号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63293961070609 138.9539843415844 0 37.63296822628593 138.95387721940048 0 37.632904089369035 138.95384310133014 0 37.632878498711314 138.95391916157485 0 37.63286902775102 138.95394947634153 0 37.63293961070609 138.9539843415844 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_b1aa982d-fb4f-4ef0-a16a-02f3ee562547">
			<gml:name>市道1158号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63293961070607 138.9539843415844 0 37.63286902775101 138.9539494763415 0 37.63280469317067 138.9541553983985 0 37.63271178909658 138.95441383872867 0 37.63264663559075 138.95457920097903 0 37.6326117386777 138.9546793728167 0 37.63259386899832 138.9547335349575 0 37.632584857509 138.95478049981517 0 37.63257699793859 138.95483053166225 0 37.632573957427134 138.95484988667613 0 37.63256659497892 138.9549131768487 0 37.632560357222516 138.95498493216456 0 37.63255273441143 138.95507261960037 0 37.63254907567778 138.95511009978574 0 37.63261260666203 138.95512054667978 0 37.63262010260833 138.95508084816794 0 37.63264815263573 138.95486359509337 0 37.63265794657574 138.95480065956767 0 37.632665211148165 138.9547627480829 0 37.63268070973585 138.9547159360016 0 37.63271442265831 138.95461904260054 0 37.632779394701295 138.95445401903336 0 37.632875945359416 138.95418461050883 0 37.63289374343496 138.9541255756475 0 37.632930433833856 138.95400411346205 0 37.63293961070607 138.9539843415844 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_2de22105-42d3-4621-8006-68a7cc6b5d20">
			<gml:name>市道1228号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63260046296189 138.9551866288822 0 37.63260621272208 138.95515440890688 0 37.63261260666205 138.95512054667978 0 37.632549075677794 138.95511009978574 0 37.632541696918096 138.95517758193304 0 37.63260046296189 138.9551866288822 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_d13f89b1-aac5-4c00-a658-34adc04a13a4">
			<gml:name>市道1228号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.632541696918096 138.95517758193304 0 37.632537797123945 138.9552074694985 0 37.63252885942263 138.95528196674985 0 37.63252315444672 138.9553126363945 0 37.632514209627345 138.95534226656096 0 37.632498667476916 138.95537695498595 0 37.63248323859846 138.95540563916154 0 37.632446619354525 138.95543827281773 0 37.6324303650762 138.9554473509516 0 37.63241818475406 138.95545146872848 0 37.632406008367944 138.95545456682467 0 37.63258150877673 138.9554384162445 0 37.632584017262644 138.95541883054204 0 37.63258869700508 138.95539699207526 0 37.63259946970112 138.9553373483623 0 37.63259479190009 138.95521858072502 0 37.63259700293442 138.9552060178385 0 37.63260046296189 138.9551866288822 0 37.632541696918096 138.95517758193304 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_b9e43a5f-9251-424a-9b40-5128df71a3b3">
			<gml:name>市道1258号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63234802531632 138.95618907859964 0 37.632293479873034 138.95640673475012 0 37.63235817226068 138.9564311500681 0 37.632422051679015 138.95617650157962 0 37.632445197815116 138.9560628894348 0 37.63246595689788 138.95596070598154 0 37.63247700632108 138.95589947793283 0 37.632492694011766 138.95580383484435 0 37.63251445631148 138.9556751450531 0 37.63238955763654 138.95563121496932 0 37.63239281625663 138.95565072252606 0 37.63239596157795 138.95567623431174 0 37.63239703356852 138.9557252999138 0 37.632399594428115 138.9557854780462 0 37.63240360831367 138.9558427192487 0 37.63241093729865 138.95588151274748 0 37.63240025574685 138.95594081688614 0 37.632389326182654 138.95599434117446 0 37.632387489057976 138.9560033939765 0 37.63235699443892 138.9561532172335 0 37.63234802531632 138.95618907859964 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_8b6d0c20-15a9-473b-aee5-14b704c0656e">
			<gml:name>市道1258号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63229347987304 138.9564067347501 0 37.632258532992154 138.9565427077952 0 37.63232650362418 138.95656764175772 0 37.63235817226068 138.9564311500681 0 37.63229347987304 138.9564067347501 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_937b0b6a-61b2-4463-ad4a-9dbccc8f6482">
			<gml:name>市道1258号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63232650362418 138.95656764175772 0 37.632258532992154 138.9565427077952 0 37.63225758021339 138.95655607138355 0 37.632254246128646 138.95657939081488 0 37.632199558980474 138.9568101883434 0 37.632195983045065 138.9568261417612 0 37.6321868142063 138.9568669869474 0 37.63218176506471 138.9568893794932 0 37.63215793453204 138.95699506573624 0 37.63213831619389 138.957080680575 0 37.632208048961395 138.95707940710753 0 37.63222209143423 138.95701811863773 0 37.63226353351647 138.9568338064837 0 37.6323184026747 138.956602556688 0 37.63232650362418 138.95656764175772 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_f8af2252-c115-4ae3-bee4-da0293c133ae">
			<gml:name>市道1183号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.632241558544756 138.95358456421894 0 37.63225031753509 138.95353283928975 0 37.63227666880229 138.95338174368706 0 37.63230768093628 138.95321356794855 0 37.632309251884905 138.95320338046776 0 37.63227807563349 138.9532031911344 0 37.63227650468489 138.95321337861097 0 37.6322467871053 138.95337295133353 0 37.63222034356736 138.95352461281874 0 37.63221066175554 138.9535819971236 0 37.632241558544756 138.95358456421894 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_f76ad5c5-24e0-4ad7-b0a7-8c1055d6cbbd">
			<gml:name>市道1228号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.632549075677794 138.95511009978574 0 37.632531515740965 138.955107273481 0 37.63244939396496 138.95509283670242 0 37.63239123262 138.95508047217686 0 37.632220497578956 138.95505257888627 0 37.63198727046047 138.9550137678157 0 37.631701185464415 138.95496591069457 0 37.63151179825011 138.95493801753426 0 37.63146465780757 138.95487167687597 0 37.63145428716438 138.9549906908435 0 37.631506251835475 138.95499769230202 0 37.631695098843224 138.95502546901835 0 37.6319918179229 138.95507293801117 0 37.632214815032526 138.95510081007245 0 37.63238528018443 138.95512858852476 0 37.63244344108711 138.95514106638453 0 37.63244120514702 138.9551600872198 0 37.632523957210566 138.95517464121565 0 37.632541696918096 138.95517758193304 0 37.632549075677794 138.95511009978574 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_61f417cf-f5f0-4ff1-a80b-45785a7b8afd">
			<gml:name>市道1157号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63229347987304 138.9564067347501 0 37.63225946988567 138.95644029019215 0 37.63220427030919 138.95643100186643 0 37.632162037924154 138.9564238322635 0 37.6321043154127 138.95641452853263 0 37.63207180688686 138.9564093445169 0 37.63203172167396 138.95640615356461 0 37.63201262516712 138.956404563876 0 37.631979926350866 138.95640198460134 0 37.63185074157421 138.95639462316367 0 37.63183992944371 138.95639444374078 0 37.63172387979647 138.95639237443035 0 37.63166387266764 138.9563913276553 0 37.63162407354213 138.9563840597113 0 37.63156231997229 138.95636861328035 0 37.63132421376616 138.95631005464915 0 37.63131377256932 138.95630715833948 0 37.631317761934696 138.95639396955136 0 37.63152175390188 138.9564431425681 0 37.631550021370295 138.95644988680513 0 37.631612944975906 138.95646568035474 0 37.63165859739781 138.95647389053772 0 37.631694277122115 138.95647456197676 0 37.63173139852207 138.95647524223494 0 37.631848889842665 138.95647732049295 0 37.631976092747365 138.9564845566509 0 37.63203887777659 138.95648958599364 0 37.632065631431175 138.95649167574288 0 37.63208139055457 138.95649415143916 0 37.632153971573494 138.9565058121207 0 37.63223528266671 138.9565201321336 0 37.6322484274366 138.95652293174905 0 37.632258532992154 138.9565427077952 0 37.63229347987304 138.9564067347501 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_43efe209-32bd-47df-a366-c62281aee8dc">
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">9020</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631638307463895 138.95668756203148 0 37.63164391479725 138.95665893164474 0 37.63157020539742 138.95663616068268 0 37.631489466068246 138.95661379994948 0 37.63147749485265 138.95661044103346 0 37.631303493808 138.95656507662616 0 37.63130076415009 138.95659519737893 0 37.63147260749228 138.95663930236347 0 37.631565228364366 138.95666490819895 0 37.631638307463895 138.95668756203148 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_53c0f6dc-99f4-4509-8679-ca34ac968181">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63152108094147 138.95348637355963 0 37.631571183894074 138.9534941060006 0 37.63157668348782 138.95343776621397 0 37.631581842574214 138.95338488686556 0 37.631589733945404 138.95330098014148 0 37.631593869610995 138.95325647869564 0 37.63160125947659 138.95320927782757 0 37.63160256011007 138.95319908880137 0 37.631546965458355 138.95319875118716 0 37.631545664824934 138.95320894020577 0 37.63154380747539 138.9532467708175 0 37.63153177217802 138.95337733153485 0 37.63152108094147 138.95348637355963 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_6017f1e8-aae1-43da-a60a-1553edb68f3c">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631571183894074 138.9534941060006 0 37.63152108094147 138.95348637355963 0 37.63151920525266 138.95350550969906 0 37.63156931580277 138.95351324332538 0 37.631571183894074 138.9534941060006 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_07fe430b-6446-4b79-96c3-741eacbe024b">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63156931580277 138.95351324332538 0 37.63151920525266 138.95350550969906 0 37.63151273368867 138.953571637004 0 37.63150203993978 138.95372815124242 0 37.63148961862212 138.95391207328387 0 37.63148216005767 138.95404730689938 0 37.631536219716814 138.95400163638715 0 37.63153987834493 138.95391725083985 0 37.631544942418515 138.95384227765413 0 37.63154714120451 138.9538094342931 0 37.63154723915303 138.95380739550532 0 37.63155220564534 138.95373434779566 0 37.63156298863676 138.95357806059363 0 37.63156931580277 138.95351324332538 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_19ca710e-0a01-4641-8767-ec1636ca95ac">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63153238692324 138.9540844346794 0 37.631536219716814 138.95400163638715 0 37.63148216005767 138.95404730689938 0 37.631474564472185 138.95414787016952 0 37.63153238692324 138.9540844346794 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_68ae6f4e-ebf1-4de4-865d-1808a03538c5">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631415760644295 138.95486596623314 0 37.63146465780754 138.95487167687597 0 37.63147731922125 138.9547193669371 0 37.6314920963047 138.95453240034104 0 37.63150940824164 138.95434227672968 0 37.63152414155442 138.95416652631488 0 37.63153238692324 138.95408443467943 0 37.631474564472185 138.95414787016952 0 37.63147397456488 138.9541606693628 0 37.63145915290176 138.9543359659271 0 37.631441932389784 138.95452575007212 0 37.63142706433953 138.95471294258752 0 37.631415760644295 138.95486596623314 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_188a1a7a-5e1d-42bd-89e6-1ba1390d15f1">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631454287164374 138.9549906908435 0 37.63146465780754 138.95487167687597 0 37.631415760644295 138.95486596623314 0 37.63141270910209 138.95490727598172 0 37.63140599625372 138.95498903683915 0 37.631454287164374 138.9549906908435 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_4a859a5a-b229-4781-80b3-cbd84075bcf5">
			<gml:name>市道1159号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63138414013311 138.95523566843042 0 37.631430660909224 138.9552523805489 0 37.63143125355767 138.9552389015955 0 37.631437846317255 138.9551649576258 0 37.631454287164374 138.9549906908435 0 37.63140599625372 138.95498903683915 0 37.631390384271974 138.95515866331226 0 37.63138414013311 138.95523566843042 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_e2e03697-321c-46bc-ac49-499c9a11cfa4">
			<gml:name>市道1156号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63142514191932 138.95532825713593 0 37.63137426945602 138.95531842964903 0 37.63137748612009 138.95532547380083 0 37.631378632123145 138.95533205212422 0 37.63137897330421 138.95533703935382 0 37.631346603510615 138.95557601555078 0 37.631324372674065 138.9558027039033 0 37.63130539499981 138.95598024036104 0 37.63130333825982 138.95599948857327 0 37.63129745453832 138.95605304290277 0 37.63127950063354 138.95619874856587 0 37.631267407619106 138.9562967912678 0 37.63131377256932 138.95630715833948 0 37.63134562773668 138.9560618346952 0 37.63135095638394 138.95601201579964 0 37.63137263993578 138.95581047642278 0 37.63139477718355 138.95558469374558 0 37.631404465650846 138.9555024979224 0 37.63142514191932 138.95532825713593 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_fd91ffd9-021f-4c09-92f4-e1694cf0cc56">
			<gml:name>市道1156号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631317761934696 138.95639396955136 0 37.63131377256932 138.95630715833948 0 37.631267407619106 138.9562967912678 0 37.63126992595464 138.9563911844386 0 37.631317761934696 138.95639396955136 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_8aa1fc4e-c2e6-47e5-9265-29be060ae4da">
			<gml:name>市道1156号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631303493808 138.95656507662616 0 37.63130476205513 138.95654004537255 0 37.63131217665261 138.95646350080287 0 37.631317761934696 138.95639396955136 0 37.63126992595464 138.9563911844386 0 37.63126723366912 138.95645824077047 0 37.63125973203918 138.9565339916718 0 37.631303493808 138.95656507662616 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_7a09f4f9-dd86-4b4a-ab76-3333e2bae798">
			<gml:name>市道1156号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63125824237373 138.9565696716269 0 37.63130076415009 138.95659519737893 0 37.631303493808 138.95656507662616 0 37.63125973203918 138.9565339916718 0 37.631258385525484 138.9565559633686 0 37.63125824237373 138.9565696716269 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_308e2f10-d781-4ca5-9b3b-4e8eedaf8605">
			<gml:name>市道1156号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63121676665053 138.95690512932748 0 37.631216476201004 138.95690761232157 0 37.6312610117982 138.95692499307998 0 37.63128639602524 138.95672223058023 0 37.63130001393664 138.95660278380132 0 37.63130076415009 138.95659519737893 0 37.63125824237373 138.9565696716269 0 37.631241826191065 138.95671368701815 0 37.63122678403858 138.95682859302175 0 37.63121676665053 138.95690512932748 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_c641c26a-0c07-436f-9c41-89c8779e1562">
			<gml:name>市道1161号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63151920525266 138.95350550969906 0 37.63152108094147 138.9534863735596 0 37.63138180815287 138.95345448337534 0 37.631308890766434 138.9534364790168 0 37.631287553783764 138.95343170412926 0 37.63128376326179 138.9534567201069 0 37.63126223188824 138.95359628651025 0 37.63124269842003 138.95370833341568 0 37.63122237176924 138.9538157301967 0 37.631210152825815 138.9539001766243 0 37.63119889402349 138.95399267306172 0 37.63121382961993 138.9539984288486 0 37.631225188556265 138.95390332713504 0 37.63123740488739 138.95381956046677 0 37.63125763925864 138.95371272959736 0 37.631277265449256 138.95360000344223 0 37.63129805857186 138.95346496447422 0 37.63130213854202 138.95345841794025 0 37.63130899477871 138.95345630692447 0 37.63137651022938 138.95347314548968 0 37.63151920525266 138.95350550969906 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_ea2b6576-4aa8-4bc3-b864-f70959e343b7">
			<gml:name>市道1160号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631474564472185 138.95414787016952 0 37.63148216005767 138.95404730689938 0 37.631455669106046 138.95407059860534 0 37.631408566981115 138.9540644204747 0 37.63137525524623 138.9540571932682 0 37.63135005294058 138.9540500154072 0 37.63121760922531 138.95399969812584 0 37.63121382961993 138.9539984288486 0 37.63119956560532 138.9540757249841 0 37.631334542251004 138.95412345192966 0 37.631363434923266 138.95413167195983 0 37.63140025812319 138.95413960035992 0 37.63144934124074 138.95414613048925 0 37.631474564472185 138.95414787016952 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_77dd2693-80db-405e-9e90-bf2aeb1a04ef">
			<gml:name>市道1160号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63119956560532 138.9540757249841 0 37.63121382961993 138.9539984288486 0 37.63119889402349 138.95399267306172 0 37.631180972380676 138.95406880510765 0 37.63119956560532 138.9540757249841 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_6e9906fd-8311-414e-899a-20c2a70ca502">
			<gml:name>市道1160号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.630919021726896 138.95396933057464 0 37.6309247109697 138.95396607953626 0 37.63093389817043 138.95396704180644 0 37.631019897102725 138.95400427357518 0 37.631140003878365 138.95405258959408 0 37.631171402486494 138.95406524346308 0 37.631180972380704 138.95406880510765 0 37.631198894023484 138.95399267306172 0 37.63118998585645 138.95398944651137 0 37.63103974015656 138.95392893761684 0 37.63095446073104 138.95389205019518 0 37.63094811079694 138.95387966207494 0 37.63094555702945 138.95386423799314 0 37.630919021726896 138.95396933057464 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_60a3c8cf-30d4-417a-8242-02126dc2902a">
			<gml:name>市道1371号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.631267407619106 138.9562967912678 0 37.6312617694222 138.95631012603351 0 37.63125850199028 138.95631622418114 0 37.631252813560465 138.9563192484621 0 37.631234079221336 138.9563172078345 0 37.631219234283996 138.9563113388382 0 37.63115421692588 138.95627774484376 0 37.63107229578702 138.956234757103 0 37.63102339182375 138.95620522718167 0 37.630981410587495 138.9561797050521 0 37.630973676777785 138.95619903177035 0 37.630911375228855 138.9561621689415 0 37.63078812804413 138.95608731877712 0 37.63074177894106 138.95604953393635 0 37.63067150793412 138.9559997067106 0 37.63058747276607 138.95594413060752 0 37.63054901824971 138.9559153446919 0 37.630521147497596 138.95589919948162 0 37.630459477018235 138.95586222768742 0 37.6304507743614 138.9558991094838 0 37.63050840193508 138.95593299757272 0 37.630535913139134 138.95594891400114 0 37.63057501151569 138.9559743049349 0 37.630659406650715 138.9560299965598 0 37.630705516101195 138.9560598490363 0 37.63073203087907 138.9560771190389 0 37.630780030326825 138.95610754961433 0 37.63085140750939 138.9561509257895 0 37.630903996584635 138.95618285739155 0 37.63096593858421 138.95621949143964 0 37.6310074706024 138.9562446709451 0 37.631057992936356 138.95627511716697 0 37.631140723250546 138.95631856308503 0 37.6312068201012 138.95635261690407 0 37.631263839149376 138.95638072374012 0 37.63126992595464 138.9563911844386 0 37.631267407619106 138.9562967912678 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_72eb5661-6461-4cac-b49c-2ee82989c644">
			<gml:name>市道1155号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63125973203918 138.9565339916718 0 37.63124389113867 138.95652936282235 0 37.631194222579346 138.95651115781538 0 37.63112899876988 138.95648436030174 0 37.63105883321763 138.95645390706377 0 37.63102313351068 138.95643510779772 0 37.63087855139741 138.956355028194 0 37.63079017647787 138.95630327713016 0 37.630708813933026 138.9562556477807 0 37.63065072147595 138.9562254953329 0 37.630593617972444 138.95619591546574 0 37.63040816972332 138.95610482378896 0 37.630405841013975 138.95610118404238 0 37.63040540538049 138.95609732927062 0 37.63039842033478 138.9561327486452 0 37.63040085841923 138.95613140397845 0 37.630403110162405 138.95613164433536 0 37.63058550096169 138.95622113121541 0 37.63070006749053 138.95628051982098 0 37.63086989460044 138.95638001412513 0 37.63101528851612 138.95645987213652 0 37.63105018648105 138.95647628724385 0 37.63112414300226 138.95650506421228 0 37.63118918703581 138.9565317473439 0 37.63123957555315 138.95655018336493 0 37.631258385525484 138.9565559633686 0 37.63125973203918 138.9565339916718 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_c6319d0e-716b-4b0e-b60a-bf79dfcb85d2">
			<gml:name>市道1172号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.630945557029456 138.95386423799312 0 37.6309479851707 138.95384204631898 0 37.63096128001063 138.95378241902293 0 37.63098469502308 138.95366858338312 0 37.63102308445186 138.95345660906534 0 37.631050966957105 138.9533056385149 0 37.63105811786272 138.95325027906293 0 37.631063064015486 138.95320589615116 0 37.6310648147371 138.9531958232299 0 37.63100038981057 138.95319543200188 0 37.63099863908902 138.95320550491448 0 37.63098652374544 138.95328655287338 0 37.63095853983984 138.9534404684377 0 37.63092070370207 138.95364916029848 0 37.630897654356374 138.95376163849326 0 37.630945557029456 138.95386423799312 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_3032159c-c982-4d31-8315-0b641205e8c3">
			<gml:name>市道1172号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.6308612046574 138.9539377086181 0 37.630919021726896 138.95396933057464 0 37.630945557029456 138.95386423799312 0 37.630897654356374 138.95376163849326 0 37.63085728842887 138.95392510871616 0 37.6308612046574 138.9539377086181 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_dc51909e-ef08-4eb6-a39d-2ad2e746ac06">
			<gml:name>市道1164号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63037364758825 138.95374734934737 0 37.63045923311414 138.95377494778208 0 37.630700136429915 138.95385628748815 0 37.630740537696646 138.95387103530987 0 37.63083978686139 138.95390710118087 0 37.630850391331485 138.9539144167517 0 37.63085728842887 138.95392510871616 0 37.63089765435638 138.95376163849326 0 37.630845810897675 138.9537933866143 0 37.63072352101325 138.95375038293233 0 37.630481537672686 138.95366869708386 0 37.630394664625314 138.95364012649318 0 37.63037364758825 138.95374734934737 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_8b0cd972-704f-40fc-8c79-68a26d216d35">
			<gml:name>市道1172号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.630919021726896 138.95396933057464 0 37.6308612046574 138.9539377086181 0 37.630862496512535 138.9539532383071 0 37.63085391450036 138.95400575638865 0 37.63080868920976 138.95421428939667 0 37.63077861095218 138.95435017712333 0 37.630774122085725 138.9543692971405 0 37.63082355227387 138.9543830841231 0 37.63082711111408 138.95436769368771 0 37.63085728211387 138.95423112665637 0 37.630898560993714 138.95404126369988 0 37.630903055910686 138.95402055749437 0 37.63091502573048 138.95397746373163 0 37.630919021726896 138.95396933057464 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_0474d8a6-11e0-4af1-9bae-6245aa33153e">
			<gml:name>市道1172号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.6307656827906 138.95440810693572 0 37.63080905940434 138.95444575937307 0 37.63082355227387 138.9543830841231 0 37.630774122085725 138.9543692971405 0 37.6307656827906 138.95440810693572 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_c49dd30b-e804-47ee-a323-a5af1a5fcc35">
			<gml:name>市道1172号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63080905940434 138.9544457593731 0 37.6307656827906 138.95440810693574 0 37.6307618320662 138.9544253047677 0 37.630728965307824 138.95458326841631 0 37.63069552801751 138.95474893269775 0 37.63066644597712 138.9548833532023 0 37.630640524878814 138.9550158668128 0 37.630688841235425 138.9550342890375 0 37.63071522292509 138.95489917230574 0 37.63074430367378 138.95476509160545 0 37.630777833274486 138.95459886129606 0 37.63080905940434 138.9544457593731 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_c5ff8829-9a2f-4f15-b858-e8bd1d7cea96">
			<gml:name>市道1163号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.6307656827906 138.95440810693574 0 37.630774122085725 138.95436929714052 0 37.63065055567975 138.95433025020714 0 37.63046191677918 138.95427177332925 0 37.6302877520023 138.95422188212999 0 37.630282082814965 138.95421992157446 0 37.63027454418155 138.95425884991073 0 37.6302812950619 138.95426070375578 0 37.63045473985618 138.95431036406345 0 37.630642478561995 138.95436860895697 0 37.6307656827906 138.95440810693574 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_0ad5696c-8bca-4bfe-89d8-8a6571884be8">
			<gml:name>市道1172号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.630430184006855 138.9557710709676 0 37.63047507065244 138.9557968112953 0 37.630523713859276 138.955608040541 0 37.630576322324934 138.95540159359183 0 37.630629024469314 138.9551941272427 0 37.63065288096035 138.95510635372392 0 37.63060501513882 138.955087934209 0 37.63058278316997 138.95517503779996 0 37.63054462852313 138.95532537753786 0 37.630530172038775 138.9553822779948 0 37.630525047899454 138.95540252698436 0 37.630479850348244 138.955579901589 0 37.630477288480854 138.95558996942302 0 37.63043292784966 138.955760551068 0 37.630430184006855 138.9557710709676 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
	<core:cityObjectMember>
		<tran:Road gml:id="tran_72a1bc5e-ef8e-43d2-9a2f-0e2c7940aa56">
			<gml:name>市道1168号線</gml:name>
			<core:creationDate>2026-03-19</core:creationDate>
			<tran:class codeSpace="../../codelists/TransportationComplex_class.xml">1040</tran:class>
			<tran:function codeSpace="../../codelists/Road_function.xml">4</tran:function>
			<tran:lod1MultiSurface>
				<gml:MultiSurface srsName="http://www.opengis.net/def/crs/EPSG/0/6697" srsDimension="3">
					<gml:surfaceMember>
						<gml:Polygon>
							<gml:exterior>
								<gml:LinearRing>
									<gml:posList>37.63343684849707 138.9534615322813 0 37.633419047467996 138.95335729964322 0 37.633418756256916 138.95336273636627 0 37.633415772351526 138.9533654374839 0 37.633256169415525 138.95334815239192 0 37.633250054916836 138.9533448294971 0 37.633247919828506 138.9533376785215 0 37.633225518045215 138.95344585878718 0 37.63322798222442 138.95343771604317 0 37.633233678417966 138.95343265208137 0 37.63323935545301 138.953432573275 0 37.633418145463025 138.9534513347575 0 37.633431109225945 138.95345435938316 0 37.63343684849707 138.9534615322813 0</gml:posList>
								</gml:LinearRing>
							</gml:exterior>
						</gml:Polygon>
					</gml:surfaceMember>
				</gml:MultiSurface>
			</tran:lod1MultiSurface>
			<uro:tranDataQualityAttribute>
				<uro:DataQualityAttribute>
					<uro:geometrySrcDescLod1 codeSpace="../../codelists/DataQualityAttribute_geometrySrcDesc.xml">000</uro:geometrySrcDescLod1>
					<uro:thematicSrcDesc codeSpace="../../codelists/DataQualityAttribute_thematicSrcDesc.xml">000</uro:thematicSrcDesc>
					<uro:publicSurveyDataQualityAttribute>
						<uro:PublicSurveyDataQualityAttribute>
							<uro:srcScaleLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_srcScale.xml">1</uro:srcScaleLod1>
							<uro:publicSurveySrcDescLod1 codeSpace="../../codelists/PublicSurveyDataQualityAttribute_publicSurveySrcDesc.xml">023</uro:publicSurveySrcDescLod1>
						</uro:PublicSurveyDataQualityAttribute>
					</uro:publicSurveyDataQualityAttribute>
				</uro:DataQualityAttribute>
			</uro:tranDataQualityAttribute>
		</tran:Road>
	</core:cityObjectMember>
</core:CityModel>