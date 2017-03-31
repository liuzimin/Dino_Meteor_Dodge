module controlgame(CLOCK_50, SW, GPIO, HEX0, HEX1, HEX2);

	output[35:0] GPIO;
	input CLOCK_50;
	input [9:0] SW;
  wire [12:0] score;
	wire charstate;
	wire gclk;
	wire vstate;
	wire cont;
  reg res;
  wire [2:0] obs1s, obs2s, obs3s;

	reg[1:0] charpos;
	wire[1:0] charposwi;

	assign charposwi = charpos;
	assign GPIO[1:0] = charposwi;
  
  rand rng(.clk(gclk), .out(vstate));
  
  incrementscore scorecounter(.clk(gclk), .score(score));
  hex_display h0(.IN(score[3:0]), .OUT(HEX0));
  hex_display h1(.IN(score[7:4]), .OUT(HEX1));
  hex_display h2(.IN(score[11:8]), .OUT(HEX2));

  obs obs1(.clk(gclk), .init(vstate), .initpos(4'b1000), .cont(cont), .state(obs1s), .reset(res));
  obs obs2(.clk(gclk), .init(vstate), .initpos(4'b0100), .cont(cont), .state(obs2s), .reset(res));
  obs obs3(.clk(gclk), .init(vstate), .initpos(4'b0000), .cont(cont), .state(obs3s), .reset(res));
  
  // Controllers for checking if LEDs are supposed to turn on when object at pos is given to it
  // e.g. cdled0 means control down led 0, which means using input from object 1, 2, 3, controls
  // whether the downwards led will be turned on (given a position that the LED represents)
  objectcontrol cdled0(.GPIO(GPIO[2]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00000));
  objectcontrol culed0(.GPIO(GPIO[3]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10000));
  objectcontrol cdled1(.GPIO(GPIO[4]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00001));
  objectcontrol culed1(.GPIO(GPIO[5]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10001));
  objectcontrol cdled2(.GPIO(GPIO[6]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00010));
  objectcontrol culed2(.GPIO(GPIO[7]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10010));
  objectcontrol cdled3(.GPIO(GPIO[8]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00011));
  objectcontrol culed3(.GPIO(GPIO[9]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10011));
  objectcontrol cdled4(.GPIO(GPIO[10]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00100));
  objectcontrol culed4(.GPIO(GPIO[11]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10100));
  objectcontrol cdled5(.GPIO(GPIO[12]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00101));
  objectcontrol culed5(.GPIO(GPIO[13]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10101));
  objectcontrol cdled6(.GPIO(GPIO[14]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00110));
  objectcontrol culed6(.GPIO(GPIO[15]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10110));
  objectcontrol cdled7(.GPIO(GPIO[16]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b00111));
  objectcontrol culed7(.GPIO(GPIO[17]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b10111));
  objectcontrol cdled8(.GPIO(GPIO[18]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b01000));
  objectcontrol culed8(.GPIO(GPIO[19]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b11000));
  objectcontrol cdled9(.GPIO(GPIO[20]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b01001));
  objectcontrol culed9(.GPIO(GPIO[21]), .obs1s(obs1s), .obs2s(obs2s), .obs3s(obs3s), .pos(5'b11001));

  // Character controller
  controlchar cc(.clk(gclk), .trigger(SW[0]), .state(charstate), .obs1(obs1s), .obs2(obs2s), .obs3(obs3s), .reset(res));

	always @(posedge gclk) begin
    if(~charstate && cont)
		begin
			charpos[0] <= 1'b0;
			charpos[1] <= 1'b1;
		end
		else
		begin
			charpos[0] <= 1'b1;
			charpos[1] <= 1'b0;
		end
    
    if(~cont && SW[9])
  	begin
      res <= 1'b1;
    end
    else
      res <= 1'b0;
	end

	clkcount clkc(.clk(CLOCK_50), .clk_out(gclk));

endmodule

module rand(clk, out);
	output out;
	input clk;
	reg[1023:0] outr;
	reg[10:0] count;
	always @(*)begin
	// 'rng'
	{outr[0], outr[1], outr[2], outr[3], outr[4], outr[5], outr[6], outr[7], outr[8], outr[9], outr[10], outr[11], outr[12], outr[13], outr[14], outr[15], outr[16], outr[17], outr[18], outr[19], outr[20], outr[21], outr[22], outr[23], outr[24], outr[25], outr[26], outr[27], outr[28], outr[29], outr[30], outr[31], outr[32], outr[33], outr[34], outr[35], outr[36], outr[37], outr[38], outr[39], outr[40], outr[41], outr[42], outr[43], outr[44], outr[45], outr[46], outr[47], outr[48], outr[49], outr[50], outr[51], outr[52], outr[53], outr[54], outr[55], outr[56], outr[57], outr[58], outr[59], outr[60], outr[61], outr[62], outr[63], outr[64], outr[65], outr[66], outr[67], outr[68], outr[69], outr[70], outr[71], outr[72], outr[73], outr[74], outr[75], outr[76], outr[77], outr[78], outr[79], outr[80], outr[81], outr[82], outr[83], outr[84], outr[85], outr[86], outr[87], outr[88], outr[89], outr[90], outr[91], outr[92], outr[93], outr[94], outr[95], outr[96], outr[97], outr[98], outr[99], outr[100], outr[101], outr[102], outr[103], outr[104], outr[105], outr[106], outr[107], outr[108], outr[109], outr[110], outr[111], outr[112], outr[113], outr[114], outr[115], outr[116], outr[117], outr[118], outr[119], outr[120], outr[121], outr[122], outr[123], outr[124], outr[125], outr[126], outr[127], outr[128], outr[129], outr[130], outr[131], outr[132], outr[133], outr[134], outr[135], outr[136], outr[137], outr[138], outr[139], outr[140], outr[141], outr[142], outr[143], outr[144], outr[145], outr[146], outr[147], outr[148], outr[149], outr[150], outr[151], outr[152], outr[153], outr[154], outr[155], outr[156], outr[157], outr[158], outr[159], outr[160], outr[161], outr[162], outr[163], outr[164], outr[165], outr[166], outr[167], outr[168], outr[169], outr[170], outr[171], outr[172], outr[173], outr[174], outr[175], outr[176], outr[177], outr[178], outr[179], outr[180], outr[181], outr[182], outr[183], outr[184], outr[185], outr[186], outr[187], outr[188], outr[189], outr[190], outr[191], outr[192], outr[193], outr[194], outr[195], outr[196], outr[197], outr[198], outr[199], outr[200], outr[201], outr[202], outr[203], outr[204], outr[205], outr[206], outr[207], outr[208], outr[209], outr[210], outr[211], outr[212], outr[213], outr[214], outr[215], outr[216], outr[217], outr[218], outr[219], outr[220], outr[221], outr[222], outr[223], outr[224], outr[225], outr[226], outr[227], outr[228], outr[229], outr[230], outr[231], outr[232], outr[233], outr[234], outr[235], outr[236], outr[237], outr[238], outr[239], outr[240], outr[241], outr[242], outr[243], outr[244], outr[245], outr[246], outr[247], outr[248], outr[249], outr[250], outr[251], outr[252], outr[253], outr[254], outr[255], outr[256], outr[257], outr[258], outr[259], outr[260], outr[261], outr[262], outr[263], outr[264], outr[265], outr[266], outr[267], outr[268], outr[269], outr[270], outr[271], outr[272], outr[273], outr[274], outr[275], outr[276], outr[277], outr[278], outr[279], outr[280], outr[281], outr[282], outr[283], outr[284], outr[285], outr[286], outr[287], outr[288], outr[289], outr[290], outr[291], outr[292], outr[293], outr[294], outr[295], outr[296], outr[297], outr[298], outr[299], outr[300], outr[301], outr[302], outr[303], outr[304], outr[305], outr[306], outr[307], outr[308], outr[309], outr[310], outr[311], outr[312], outr[313], outr[314], outr[315], outr[316], outr[317], outr[318], outr[319], outr[320], outr[321], outr[322], outr[323], outr[324], outr[325], outr[326], outr[327], outr[328], outr[329], outr[330], outr[331], outr[332], outr[333], outr[334], outr[335], outr[336], outr[337], outr[338], outr[339], outr[340], outr[341], outr[342], outr[343], outr[344], outr[345], outr[346], outr[347], outr[348], outr[349], outr[350], outr[351], outr[352], outr[353], outr[354], outr[355], outr[356], outr[357], outr[358], outr[359], outr[360], outr[361], outr[362], outr[363], outr[364], outr[365], outr[366], outr[367], outr[368], outr[369], outr[370], outr[371], outr[372], outr[373], outr[374], outr[375], outr[376], outr[377], outr[378], outr[379], outr[380], outr[381], outr[382], outr[383], outr[384], outr[385], outr[386], outr[387], outr[388], outr[389], outr[390], outr[391], outr[392], outr[393], outr[394], outr[395], outr[396], outr[397], outr[398], outr[399], outr[400], outr[401], outr[402], outr[403], outr[404], outr[405], outr[406], outr[407], outr[408], outr[409], outr[410], outr[411], outr[412], outr[413], outr[414], outr[415], outr[416], outr[417], outr[418], outr[419], outr[420], outr[421], outr[422], outr[423], outr[424], outr[425], outr[426], outr[427], outr[428], outr[429], outr[430], outr[431], outr[432], outr[433], outr[434], outr[435], outr[436], outr[437], outr[438], outr[439], outr[440], outr[441], outr[442], outr[443], outr[444], outr[445], outr[446], outr[447], outr[448], outr[449], outr[450], outr[451], outr[452], outr[453], outr[454], outr[455], outr[456], outr[457], outr[458], outr[459], outr[460], outr[461], outr[462], outr[463], outr[464], outr[465], outr[466], outr[467], outr[468], outr[469], outr[470], outr[471], outr[472], outr[473], outr[474], outr[475], outr[476], outr[477], outr[478], outr[479], outr[480], outr[481], outr[482], outr[483], outr[484], outr[485], outr[486], outr[487], outr[488], outr[489], outr[490], outr[491], outr[492], outr[493], outr[494], outr[495], outr[496], outr[497], outr[498], outr[499], outr[500], outr[501], outr[502], outr[503], outr[504], outr[505], outr[506], outr[507], outr[508], outr[509], outr[510], outr[511], outr[512], outr[513], outr[514], outr[515], outr[516], outr[517], outr[518], outr[519], outr[520], outr[521], outr[522], outr[523], outr[524], outr[525], outr[526], outr[527], outr[528], outr[529], outr[530], outr[531], outr[532], outr[533], outr[534], outr[535], outr[536], outr[537], outr[538], outr[539], outr[540], outr[541], outr[542], outr[543], outr[544], outr[545], outr[546], outr[547], outr[548], outr[549], outr[550], outr[551], outr[552], outr[553], outr[554], outr[555], outr[556], outr[557], outr[558], outr[559], outr[560], outr[561], outr[562], outr[563], outr[564], outr[565], outr[566], outr[567], outr[568], outr[569], outr[570], outr[571], outr[572], outr[573], outr[574], outr[575], outr[576], outr[577], outr[578], outr[579], outr[580], outr[581], outr[582], outr[583], outr[584], outr[585], outr[586], outr[587], outr[588], outr[589], outr[590], outr[591], outr[592], outr[593], outr[594], outr[595], outr[596], outr[597], outr[598], outr[599], outr[600], outr[601], outr[602], outr[603], outr[604], outr[605], outr[606], outr[607], outr[608], outr[609], outr[610], outr[611], outr[612], outr[613], outr[614], outr[615], outr[616], outr[617], outr[618], outr[619], outr[620], outr[621], outr[622], outr[623], outr[624], outr[625], outr[626], outr[627], outr[628], outr[629], outr[630], outr[631], outr[632], outr[633], outr[634], outr[635], outr[636], outr[637], outr[638], outr[639], outr[640], outr[641], outr[642], outr[643], outr[644], outr[645], outr[646], outr[647], outr[648], outr[649], outr[650], outr[651], outr[652], outr[653], outr[654], outr[655], outr[656], outr[657], outr[658], outr[659], outr[660], outr[661], outr[662], outr[663], outr[664], outr[665], outr[666], outr[667], outr[668], outr[669], outr[670], outr[671], outr[672], outr[673], outr[674], outr[675], outr[676], outr[677], outr[678], outr[679], outr[680], outr[681], outr[682], outr[683], outr[684], outr[685], outr[686], outr[687], outr[688], outr[689], outr[690], outr[691], outr[692], outr[693], outr[694], outr[695], outr[696], outr[697], outr[698], outr[699], outr[700], outr[701], outr[702], outr[703], outr[704], outr[705], outr[706], outr[707], outr[708], outr[709], outr[710], outr[711], outr[712], outr[713], outr[714], outr[715], outr[716], outr[717], outr[718], outr[719], outr[720], outr[721], outr[722], outr[723], outr[724], outr[725], outr[726], outr[727], outr[728], outr[729], outr[730], outr[731], outr[732], outr[733], outr[734], outr[735], outr[736], outr[737], outr[738], outr[739], outr[740], outr[741], outr[742], outr[743], outr[744], outr[745], outr[746], outr[747], outr[748], outr[749], outr[750], outr[751], outr[752], outr[753], outr[754], outr[755], outr[756], outr[757], outr[758], outr[759], outr[760], outr[761], outr[762], outr[763], outr[764], outr[765], outr[766], outr[767], outr[768], outr[769], outr[770], outr[771], outr[772], outr[773], outr[774], outr[775], outr[776], outr[777], outr[778], outr[779], outr[780], outr[781], outr[782], outr[783], outr[784], outr[785], outr[786], outr[787], outr[788], outr[789], outr[790], outr[791], outr[792], outr[793], outr[794], outr[795], outr[796], outr[797], outr[798], outr[799], outr[800], outr[801], outr[802], outr[803], outr[804], outr[805], outr[806], outr[807], outr[808], outr[809], outr[810], outr[811], outr[812], outr[813], outr[814], outr[815], outr[816], outr[817], outr[818], outr[819], outr[820], outr[821], outr[822], outr[823], outr[824], outr[825], outr[826], outr[827], outr[828], outr[829], outr[830], outr[831], outr[832], outr[833], outr[834], outr[835], outr[836], outr[837], outr[838], outr[839], outr[840], outr[841], outr[842], outr[843], outr[844], outr[845], outr[846], outr[847], outr[848], outr[849], outr[850], outr[851], outr[852], outr[853], outr[854], outr[855], outr[856], outr[857], outr[858], outr[859], outr[860], outr[861], outr[862], outr[863], outr[864], outr[865], outr[866], outr[867], outr[868], outr[869], outr[870], outr[871], outr[872], outr[873], outr[874], outr[875], outr[876], outr[877], outr[878], outr[879], outr[880], outr[881], outr[882], outr[883], outr[884], outr[885], outr[886], outr[887], outr[888], outr[889], outr[890], outr[891], outr[892], outr[893], outr[894], outr[895], outr[896], outr[897], outr[898], outr[899], outr[900], outr[901], outr[902], outr[903], outr[904], outr[905], outr[906], outr[907], outr[908], outr[909], outr[910], outr[911], outr[912], outr[913], outr[914], outr[915], outr[916], outr[917], outr[918], outr[919], outr[920], outr[921], outr[922], outr[923], outr[924], outr[925], outr[926], outr[927], outr[928], outr[929], outr[930], outr[931], outr[932], outr[933], outr[934], outr[935], outr[936], outr[937], outr[938], outr[939], outr[940], outr[941], outr[942], outr[943], outr[944], outr[945], outr[946], outr[947], outr[948], outr[949], outr[950], outr[951], outr[952], outr[953], outr[954], outr[955], outr[956], outr[957], outr[958], outr[959], outr[960], outr[961], outr[962], outr[963], outr[964], outr[965], outr[966], outr[967], outr[968], outr[969], outr[970], outr[971], outr[972], outr[973], outr[974], outr[975], outr[976], outr[977], outr[978], outr[979], outr[980], outr[981], outr[982], outr[983], outr[984], outr[985], outr[986], outr[987], outr[988], outr[989], outr[990], outr[991], outr[992], outr[993], outr[994], outr[995], outr[996], outr[997], outr[998], outr[999], outr[1000], outr[1001], outr[1002], outr[1003], outr[1004], outr[1005], outr[1006], outr[1007], outr[1008], outr[1009], outr[1010], outr[1011], outr[1012], outr[1013], outr[1014], outr[1015], outr[1016], outr[1017], outr[1018], outr[1019], outr[1020], outr[1021], outr[1022], outr[1023]} <= {1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b1};
	end
	always @(posedge clk) begin
		count <= count + 1'b1;
	end

	assign out = outr[count];
endmodule

module controlchar(clk, trigger, state, cont, obs1, obs2, obs3, reset);
	input clk, trigger, reset;
	output state;
  reg[26:0] count;
	reg char_state, contr;
	output cont;
  input[4:0] obs1, obs2, obs3;

	localparam	chardown	= 1'b0,
				charup		= 1'b1;

	always @(posedge clk)
		begin: moving_char
    		case(char_state)
    			chardown: char_state = trigger && cont ? charup : chardown;
    			charup: char_state = trigger && cont ? chardown : charup;
    			default: char_state = chardown;
    		endcase
      if(obs1[4] == char_state || obs2[4] == char_state || obs3[4] == char_state)
			begin
        if(obs1[3:0] == 4'b1011 || obs2[3:0] == 4'b1011 || obs3[3:0] == 4'b1011)
					contr = 1'b1;
        else if(obs1[4] == 4'b1100 || obs2[4] == 4'b1100 || obs3[4] == 4'b1100)
					contr = 1'b1;
			end

      if(reset) begin
        char_state = 1'b0;
				contr = 1'b1;
      end
		end

  assign cont = contr;
	assign state = char_state;
endmodule

module objectcontrol(GPIO, obs1s, obs2s, obs3s, pos);
 	output GPIO;
  reg GPIOt;
 	input[4:0] obs1s, obs2s, obs3s, pos;

 	always@(*) begin
   	if(obs1s == pos || obs1s == pos - 1'b1)
     	GPIOt <= 1'b1;
   	else if(obs2s == pos || obs2s == pos - 1'b1)
     	GPIOt <= 1'b1;
   	else if(obs3s == pos ||obs2s == pos - 1'b1)
     	GPIOt <= 1'b1;
   	else
     	GPIOt <= 1'b0;
  end
  assign GPIO = GPIOt;
endmodule

module obs(clk, init, initpos, cont, state, reset);
	input clk, cont, init, reset;
  input [3:0] initpos;
  output[4:0] state;
  reg [3:0] cstate;
	reg vstate;

  assign state[3:0] = cstate;
  assign state[4] = vstate;

	always @(posedge clk) begin
    if(cstate == 4'd12) // find max
			cstate <= 1'b0;
    else if(~cont) begin
			cstate <= cstate + 1;
		end

    if(cstate == 1'b0 && cont)
		begin
			vstate <= init; // figure out with init
		end
    
    if(reset)
      cstate <= initpos;
	end
endmodule
   
module incrementscore(clk, score);
  input clk;
  output[12:0] score;
  reg[2:0] count;
  reg[12:0] tempscore;
  always @(posedge clk)
    begin
      count <= count + 1;
    if(count == 4'd4)
      begin
      tempscore <= tempscore + 1'b1;
      count <= 1'b0;
      end
    end
  assign score = tempscore;
endmodule

module hex_display(IN, OUT);
  input [3:0] IN;
   output reg [7:0] OUT;

   always @(*)
   begin
      case(IN[3:0])
          4'b0000: OUT = 7'b1000000;
          4'b0001: OUT = 7'b1111001;
          4'b0010: OUT = 7'b0100100;
          4'b0011: OUT = 7'b0110000;
          4'b0100: OUT = 7'b0011001;
          4'b0101: OUT = 7'b0010010;
          4'b0110: OUT = 7'b0000010;
          4'b0111: OUT = 7'b1111000;
          4'b1000: OUT = 7'b0000000;
          4'b1001: OUT = 7'b0011000;
          4'b1010: OUT = 7'b0001000;
          4'b1011: OUT = 7'b0000011;
          4'b1100: OUT = 7'b1000110;
          4'b1101: OUT = 7'b0100001;
          4'b1110: OUT = 7'b0000110;
          4'b1111: OUT = 7'b0001110;

          default: OUT = 7'b0111111;
      endcase

    end
endmodule
module clkcount(clk, clk_out);
	reg[26:0] count;
	output clk_out;
	reg out;
	input clk;

	assign clk_out = out;

	always @(posedge clk) begin
		out <= 1'b0;
		if(count == 27'd12499999)
			begin
	            count <= 27'b000000000000000000000000000;
	            out <= 1'b1;
	        end
        else
        	count <= count + 1;
	end
endmodule
