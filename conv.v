`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/31/2020 10:09:05 PM
// Design Name: 
// Module Name: conv
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module conv(
input        i_clk,
input [71:0] i_pixel_data,
input        i_pixel_data_valid,
output reg [7:0] o_convolved_data,
output reg   o_convolved_data_valid
    );
    
integer i; 
reg [7:0] kernel [8:0];
reg [7:0] multData[8:0];
reg [8:0] sumDataInt;
reg [8:0] sumData;
reg multDataValid;
reg sumDataValid;
reg convolved_data_valid;
wire [7:0]p1,p2,p3,p4,p5,p6,p7,p8,p9;

initial
begin
    for(i=0;i<9;i=i+1)
    begin
       if (i==0)
       begin
        kernel[i] = 8'b11101000;
       end
       if (i==1)
       begin
        kernel[i] = 8'b00110100;
       end
       if (i==2)
       begin
        kernel[i] = 8'b10111010;
       end
       if (i==3)
       begin
        kernel[i] = 8'b01110001;
       end
       if (i==4)
       begin
        kernel[i] = 8'b00000000; //doesn't matter what value is stored because for all Gabor filter kernels, this is 1
       end
       if (i==5)
       begin
        kernel[i] = 8'b01110001;
       end
       if (i==6)
       begin
        kernel[i] = 8'b10111010;
       end
       if (i==7)
       begin
        kernel[i] = 8'b00110100;
       end
       if (i==8)
       begin
        kernel[i] = 8'b11101000;
       end
    end
end 


always @(posedge i_clk)
begin
    multData[4] <= kernel[4]; //instead of multipying with 1, the pixel value is directly stored
    multData[0] <= p1;
    multData[1] <= p2;
    multData[2] <= p3;
    multData[3] <= p4;
    multData[5] <= p6;
    multData[6] <= p7;
    multData[7] <= p8;
    multData[8] <= p9;
    multDataValid <= i_pixel_data_valid;
end

always @(*)
begin
    sumDataInt = 0;
    for(i=0;i<9;i=i+1)
    begin
    
       if(i==0 || i==1 || i==7 || i==8)
       begin
        sumDataInt = sumDataInt - multData[i];
       end
       else
       begin
       
        sumDataInt = sumDataInt + multData[i];
       end
    end
    if (sumDataInt[8]==1'b1)
    begin
      sumDataInt = 8'b11111111;
    end
end

always @(posedge i_clk)
begin
    sumData <= sumDataInt;
    sumDataValid <= multDataValid;
end
    
always @(posedge i_clk)
begin
    o_convolved_data <= sumData;
    o_convolved_data_valid <= sumDataValid;
end
mult obj1(.m1(i_pixel_data[0*8+:8]),.m2(kernel[0]),.pr(p1));
mult obj2(.m1(i_pixel_data[1*8+:8]),.m2(kernel[1]),.pr(p2));
mult obj3(.m1(i_pixel_data[2*8+:8]),.m2(kernel[2]),.pr(p3));
mult obj4(.m1(i_pixel_data[3*8+:8]),.m2(kernel[3]),.pr(p4));

mult obj6(.m1(i_pixel_data[5*8+:8]),.m2(kernel[5]),.pr(p6));
mult obj7(.m1(i_pixel_data[6*8+:8]),.m2(kernel[6]),.pr(p7));
mult obj8(.m1(i_pixel_data[7*8+:8]),.m2(kernel[7]),.pr(p8));
mult obj9(.m1(i_pixel_data[8*8+:8]),.m2(kernel[8]),.pr(p9));
    
endmodule
module mult(m1,m2,pr);

input [7:0] m1;
input [7:0] m2;
output [7:0]pr;
wire [15:0]p;    
wire [7:0]pp1,pp2,pp3,pp4,pp5,pp6,pp7,pp8,pp9;
wire s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25,s26,s27,s28,s29,s30,s31,s32,s33,s34,s35,s36,s37,s38,s39,s40,s41,s42;
wire p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17,p18,p19,p20,p21,p22,p23,p24,p25,p26,p27,p28,p29,p30,p31,p32,p33,p34,p35,p36,p37,p38,p39,p40,p41,p42;
wire g0,g1,g2,g3,g4,g5,g6,g7,g8,g9,g10,g11,g12,g13,g14,g15,g16,g17,g18,g19,g20,g21,g22,g23,g24,g25,g26,g27,g28,g29,g30,g31,g32,g33,g34,g35,g36,g37,g38,g39,g40,g41,g42;
wire c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18,c19,c20,c21,c22,c23,c24,c25,c26,c27,c28,c29,c30,c31,c32,c33,c34,c35,c36,c37,c38,c39,c40,c41,c42,c43;
wire w0,w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13;

assign pp1[0] = m1[0]&m2[0];
assign pp2[0] = m1[0]&m2[1];
assign pp3[0] = m1[0]&m2[2];
assign pp4[0] = m1[0]&m2[3];
assign pp5[0] = m1[0]&m2[4];
assign pp6[0] = m1[0]&m2[5];
assign pp7[0] = m1[0]&m2[6];
assign pp8[0] = m1[0]&m2[7];

assign pp1[1] = m1[1]&m2[0];
assign pp2[1] = m1[1]&m2[1];
assign pp3[1] = m1[1]&m2[2];
assign pp4[1] = m1[1]&m2[3];
assign pp5[1] = m1[1]&m2[4];
assign pp6[1] = m1[1]&m2[5];
assign pp7[1] = m1[1]&m2[6];
assign pp8[1] = m1[1]&m2[7];

assign pp1[2] = m1[2]&m2[0];
assign pp2[2] = m1[2]&m2[1];
assign pp3[2] = m1[2]&m2[2];
assign pp4[2] = m1[2]&m2[3];
assign pp5[2] = m1[2]&m2[4];
assign pp6[2] = m1[2]&m2[5];
assign pp7[2] = m1[2]&m2[6];
assign pp8[2] = m1[2]&m2[7];

assign pp1[3] = m1[3]&m2[0];
assign pp2[3] = m1[3]&m2[1];
assign pp3[3] = m1[3]&m2[2];
assign pp4[3] = m1[3]&m2[3];
assign pp5[3] = m1[3]&m2[4];
assign pp6[3] = m1[3]&m2[5];
assign pp7[3] = m1[3]&m2[6];
assign pp8[3] = m1[3]&m2[7];

assign pp1[4] = m1[4]&m2[0];
assign pp2[4] = m1[4]&m2[1];
assign pp3[4] = m1[4]&m2[2];
assign pp4[4] = m1[4]&m2[3];
assign pp5[4] = m1[4]&m2[4];
assign pp6[4] = m1[4]&m2[5];
assign pp7[4] = m1[4]&m2[6];
assign pp8[4] = m1[4]&m2[7];

assign pp1[5] = m1[5]&m2[0];
assign pp2[5] = m1[5]&m2[1];
assign pp3[5] = m1[5]&m2[2];
assign pp4[5] = m1[5]&m2[3];
assign pp5[5] = m1[5]&m2[4];
assign pp6[5] = m1[5]&m2[5];
assign pp7[5] = m1[5]&m2[6];
assign pp8[5] = m1[5]&m2[7];

assign pp1[6] = m1[6]&m2[0];
assign pp2[6] = m1[6]&m2[1];
assign pp3[6] = m1[6]&m2[2];
assign pp4[6] = m1[6]&m2[3];
assign pp5[6] = m1[6]&m2[4];
assign pp6[6] = m1[6]&m2[5];
assign pp7[6] = m1[6]&m2[6];
assign pp8[6] = m1[6]&m2[7];

assign pp1[7] = m1[7]&m2[0];
assign pp2[7] = m1[7]&m2[1];
assign pp3[7] = m1[7]&m2[2];
assign pp4[7] = m1[7]&m2[3];
assign pp5[7] = m1[7]&m2[4];
assign pp6[7] = m1[7]&m2[5];
assign pp7[7] = m1[7]&m2[6];
assign pp8[7] = m1[7]&m2[7];

assign p[0] = pp1[0];
half_adder h1(p[1],w1,pp1[1],pp2[0]);

//start of cla 1st row
assign p0 = pp1[2] ^ pp2[1];
assign g0 = pp1[2] & pp2[1];
assign s0 = p0 ^ w1;
assign c1 = g0|(p0&w1);

assign p1 = pp1[3] ^ pp2[2];
assign g1 = pp1[3] & pp2[2];
assign s1 = p1 ^ c1;
assign c2 = g1|(p1&c1);

assign p2 = pp1[4] ^ pp2[3];
assign g2 = pp1[4] & pp2[3];
assign s2 = p2 ^ c2;
assign c3 = g2|(p2&c2);

assign p3 = pp1[5] ^ pp2[4];
assign g3 = pp1[5] & pp2[4];
assign s3 = p3 ^ c3;
assign c4 = g3|(p3&c3);

assign p4 = pp1[6] ^ pp2[5];
assign g4 = pp1[6] & pp2[5];
assign s4 = p4 ^ c4;
assign c5 = g4|(p4&c4);

assign p5 = pp1[7] ^ pp2[6];
assign g5 = pp1[7] & pp2[6];
assign s5 = p5 ^ c5;
assign c6 = g5|(p5&c5);

assign p6 = pp2[7] ^ pp3[6];
assign g6 = pp2[7] & pp3[6];
assign s6 = p6 ^ c6;
assign c7 = g6|(p6&c6);

assign p7 = pp3[7] ^ pp4[6];
assign g7 = pp3[7] & pp4[6];
assign s7 = p7 ^ c7;
assign c8 = g7|(p7&c7);

assign p8 = pp4[7] ^ pp5[6];
assign g8 = pp4[7] & pp5[6];
assign s8 = p8 ^ c8;
assign c9 = g8|(p8&c8);

assign p9 = pp5[7] ^ pp6[6];
assign g9 = pp5[7] & pp6[6];
assign s9 = p9 ^ c9;
assign c10 = g9|(p9&c9);

assign p10 = pp6[7] ^ pp7[6];
assign g10 = pp6[7] & pp7[6];
assign s10 = p10 ^ c10;
assign c11 = g10|(p10&c10);

assign p11 = pp7[7] ^ pp8[6];
assign g11 = pp7[7] & pp8[6];
assign s11 = p11 ^ c11;
assign c12 = g11|(p11&c11);



//end of cla first row

half_adder h2(p[2],w2,s0,pp3[0]);

//start of cla second row
assign p13 = s1 ^ pp3[1];
assign g13 = s1 & pp3[1];
assign s13 = p13 ^ w2;
assign c14 = g13|(p13&w2);

assign p14 = s2 ^ pp3[2];
assign g14 = s2 & pp3[2];
assign s14 = p14 ^ c14;
assign c15 = g14|(p14&c14);

assign p15 = s3 ^ pp3[3];
assign g15 = s3 & pp3[3];
assign s15 = p15 ^ c15;
assign c16 = g15|(p15&c15);

assign p16 = s4 ^ pp3[4];
assign g16 = s4 & pp3[4];
assign s16 = p16 ^ c16;
assign c17 = g16|(p16&c16);

assign p17 = s5 ^ pp3[5];
assign g17 = s5 & pp3[5];
assign s17 = p17 ^ c17;
assign c18 = g17|(p17&c17);

assign p18 = s6 ^ pp4[5];
assign g18 = s6 & pp4[5];
assign s18 = p18 ^ c18;
assign c19 = g18|(p18&c18);

assign p19 = s7 ^ pp5[5];
assign g19 = s7 & pp5[5];
assign s19 = p19 ^ c19;
assign c20 = g19|(p19&c19);

assign p20 = s8 ^ pp6[5];
assign g20 = s8 & pp6[5];
assign s20 = p20 ^ c20;
assign c21 = g20|(p20&c20);

assign p21 = s9 ^ pp7[5];
assign g21 = s9 & pp7[5];
assign s21 = p21 ^ c21;
assign c22 = g21|(p21&c21);

assign p22 = s10 ^ pp8[5];
assign g22 = s10 & pp8[5];
assign s22 = p22 ^ c22;
assign c23 = g22|(p22&c22);
//end of cla second row

half_adder ha3(p[3],w3,pp4[0],s13);

//start of cla third row
assign p23 = s14 ^ pp4[1];
assign g23 = s14 & pp4[1];
assign s23 = p23 ^ w3;
assign c24 = g23|(p23&w3);

assign p24 = s15 ^ pp4[2];
assign g24 = s15 & pp4[2];
assign s24 = p24 ^ c24;
assign c25 = g24|(p24&c24);

assign p25 = s16 ^ pp4[3];
assign g25 = s16 & pp4[3];
assign s25 = p25 ^ c25;
assign c26 = g25|(p25&c25);

assign p26 = s17 ^ pp4[4];
assign g26 = s17 & pp4[4];
assign s26 = p26 ^ c26;
assign c27 = g26|(p26&c26);

assign p27 = s18 ^ pp5[4];
assign g27 = s18 & pp5[4];
assign s27 = p27 ^ c27;
assign c28 = g27|(p27&c27);

assign p28 = s19 ^ pp6[4];
assign g28 = s19 & pp6[4];
assign s28 = p28 ^ c28;
assign c29 = g28|(p28&c28);

assign p29 = s20 ^ pp7[4];
assign g29 = s20 & pp7[4];
assign s29 = p29 ^ c29;
assign c30 = g29|(p29&c29);

assign p30 = s21 ^ pp8[4];
assign g30 = s21 & pp8[4];
assign s30 = p30 ^ c30;
assign c31 = g30|(p30&c30);
//end of cla 3rd row

half_adder h4(p[4],w4,pp5[0],s23);

//start of cla 4th row
assign p31 = s24 ^ pp5[1];
assign g31 = s24 & pp5[1];
assign s31 = p31 ^ w4;
assign c32 = g31|(p31&w4);

assign p32 = s25 ^ pp5[2];
assign g32 = s25 & pp5[2];
assign s32 = p32 ^ c32;
assign c33 = g32|(p32&c32);

assign p33 = s26 ^ pp5[3];
assign g33 = s26 & pp5[3];
assign s33 = p33 ^ c33;
assign c34 = g33|(p33&c33);

assign p34 = s27 ^ pp6[3];
assign g34 = s27 & pp6[3];
assign s34 = p34 ^ c34;
assign c35 = g34|(p34&c34);

assign p35 = s28 ^ pp7[3];
assign g35 = s28 & pp7[3];
assign s35 = p35 ^ c35;
assign c36 = g35|(p35&c35);

assign p36 = s29 ^ pp8[3];
assign g36 = s29 & pp8[3];
assign s36 = p36 ^ c36;
assign c37 = g36|(p36&c36);
//end of cla 4th row

half_adder h5(p[5],w5,s31,pp6[0]);
//start of cla 5th row
assign p37 = s32 ^ pp6[1];
assign g37 = s32 & pp6[1];
assign s37 = p37 ^ w5;
assign c38 = g37|(p37&w5);

assign p38 = s33 ^ pp6[2];
assign g38 = s33 & pp6[2];
assign s38 = p38 ^ c38;
assign c39 = g38|(p38&c38);

assign p39 = s34 ^ pp7[2];
assign g39 = s34 & pp7[2];
assign s39 = p39 ^ c39;
assign c40 = g39|(p39&c39);

assign p40 = s35 ^ pp8[2];
assign g40 = s35 & pp8[2];
assign s40 = p40 ^ c40;
assign c41 = g40|(p40&c40);
//end of cla 5th row

half_adder h6(p[6],w6,pp7[0],s37);
//start of cla 6th row

assign p41 = s38 ^ pp7[1];
assign g41 = s38 & pp7[1];
assign s41 = p41 ^ w6;
assign c42 = g41|(p41&w6);

assign p42 = s39 ^ pp8[1];
assign g42 = s39 & pp8[1];
assign s42 = p42 ^ c42;
assign c43 = g42|(p42&c42);
//end of cla 6th row

half_adder h7(p[7],w7,s41,pp8[0]);
half_adder h8(p[8],w8,w7,s42);
full_adder f1(p[9],w9,w8,c43,s40);
full_adder f2(p[10],w10,w9,c41,s36);
full_adder f3(p[11],w11,w10,c37,s30);
full_adder f4(p[12],w12,w11,c31,s22);
full_adder f5(p[13],w13,w12,c23,s11);
full_adder f6(p[14],p[15],w13,c12,pp8[7]);

assign pr = p[15:8];


endmodule

module full_adder(sum,cout,a,b,cin);
	output sum,cout;
	input a,b,cin;
	assign sum = a ^ b ^ cin;
	assign cout = (a & b) | (cin&(a^b));
endmodule

module half_adder(sum,cout,a,b);
   output sum,cout;
	input a,b;
	assign sum = a^b;
	assign cout = a&b;
endmodule
