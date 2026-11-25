


INSTRUCTIONS



This COMPLETE MR Series, Miniature Deep Groove Lever Bearing Library contains the following bearings in both SLDPRT and STEP formats:

MR31-2RS (1x3x1.5mm)
MR41-2RS (1.2x4x1.8mm)
MR52-2RS (2x5x2mm)
MR62-2RS (2x6x2.5mm)
MR63-2RS (3x6x2mm)
MR72-2RS (2x7x2.5mm)
MR74-2RS (4x7x2mm)
MR82-2RS (2.5x8x2.5mm)
MR83-2RS (3x8x2.5mm)
MR84-2RS (4x8x2mm)
MR85-2RS (5x8x2mm)
MR93-2RS (3x9x2.5mm)
MR95-2RS (5x9x2.5mm)
MR104-2RS (4x10x3mm)
MR105-2RS (5x10x3mm)
MR106-2RS (6x10x2.5mm)
MR117-2RS (7x11x2.5mm)
MR126-2RS (6x12x3mm)
MR128-2RS (8x12x2.5mm)
MR137-2RS (7x13x3mm)
MR148-2RS (8x14x3.5mm)



The models are usefully simplified, parameterized representations, which allow you to create any like bearing type in seconds, by little more than modifying three equation values:

_i__BORE
_i__OUTSIDE_DIAMETER
_i__WIDTH



Thus to create a further bearing model from any of the SLDPRT files, you may:

1. 	Save a copy of SLDPRT source file to an appropriate model name;

2.	open that file; right-click "Equations" in your feature manager; and select "Manage equations" from the responding popup;

3.	assign your further model's required values to the _i__BORE, _i__OUTSIDE_DIAMETER, and _i__WIDTH fields.

4.	OK-close your equations interface, save your file; and you have your new bearing.



To apply appropriate indicia to your new bearing model, first note the equation value of parameter "_output__FONT_SIZE_mm." This indicates the calculated font size for your input values. You will have to manually assign this font-size value in the following steps:

5.	Expand the bottom INDICIA folder in your file's feature manager;

6.	expand the "Cut-Extrude SEAL LABEL 1" feature within that folder and open its sketch, "Sketch TEXT RADIUS 1";

7.	click-select the text in that sketch, and when highlighting indicates the text has successfully been selected, double-click the selected text to open the text feature in your feature manager;

8.	input your desired text indicia into the text field;

6.	click the Font button below, and assign the value of _output__FONT_SIZE_mm to your text height field (which must remain toggled to "units");

7.	OK-close the font dialog and sketch, and apply the same steps 5-7 to Sketch TEXT RADIUS 2 of Cut-Extrude SEAL LABEL 2, and you have your new bearing.



An additional _i_optional__TEXT_RADIUS_ADJUSTMENT input parameter may further be used to adjust the radius upon which "seal label" indicia are extrude-cut into the opposing seal surfaces. The input value thus moves the radius outward if positive, or inward if negative, or not at all if zero (which is the default).

Obvious relationships in the remaining formulae make it possible to further customize these files to generate further bearing formats. While the units of the supplied prototyping file (p  BEARING, m_MODEL, TYPE_____proto.SLDPRT) are set to millimeters, you may further set the units otherwise to generate other model designations or families of bearings in any further units.



NOTES

In order to eliminate the redundant overhead of redundantly representing bearing internals (as may be especially costly in large assemblies), only exterior surfaces are represented in these models. Note further that aside from the bore, OD, and width parameters, the further features generated comply only to the general range of possible manufactured products, in which respects the manufacturers too are granted such leeway. In other words, the models conform to the ISO standard in its own mere fundamental terms of bore, outside diameter, and maximum chamfer dimensions (which in CAD work are regularly referred to instead as radii or fillets).

Should you have the abnormal need for more specific compliance with a particular manufactured or prototypical product, equation parameters may easily be adjusted or amended appropriately.









