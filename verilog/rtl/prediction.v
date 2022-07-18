// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module prediction #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    //input wb_rst_i,
    //input wbs_stb_i,
    //input wbs_cyc_i,
    //input wbs_we_i,
    //input [3:0] wbs_sel_i,
    //input [31:0] wbs_dat_i,
    //input [31:0] wbs_adr_i,
    //output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    //input  [127:0] la_data_in,
    //output [127:0] la_data_out,
    //input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out
    //output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    //output [2:0] irq
);
    wire clk;
    //wire rst;
    //wire rd_en;
    //wire wr_en;
    
    
    wire [1:0] mat_sel;
    wire [1:0] adr;
    //wire [10:0] in_or_w1;
    wire [20:0] w2;
    wire [2:0] decision;
    
    
    
    
    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    //wire [`MPRJ_IO_PADS-1:0] io_oeb; 
    
    
   // wire [7:0] rout_data; 
    //wire [7:0] wdata;
    //wire [2:0] addr;
    //wire [BITS-1:0] count;

    //wire valid;
    //wire [3:0] wstrb;
    //wire [31:0] la_write;

    // WB MI A
    //assign valid = wbs_cyc_i && wbs_stb_i; 
    //assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    
    assign clk=wb_clk_i;
     //assign wdata = wbs_dat_i;
    //assign addr= wbs_adr_i;
    //assign wdata = io_in[15:8];
    assign mat_sel= io_in[26:25];
    assign adr=io_in[23:22];
    
    //assign in_or_w1=io_in[10:0];
    assign w2=io_in[20:0];
    //assign addr=io_in[7:5];
    //assign addr= wbs_adr_i;
    //assign wbs_dat_i=wdata;
   
    //assign io_in=wdata[7:0];
    assign io_out[31:29]=decision;
    assign wbs_dat_o = decision;
   

    // IO
    //assign io_out = count;
    //assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    //assign irq = 3'b000;	// Unused

    // LA
    //assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    //assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    //assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    //assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    mul_matr #(
        .BITS(BITS)
    ) mul_matr(
        .clk(clk),
        //.reset(rst),
        //.ready(wbs_ack_o)
        //.valid(valid),
        .mat_sel(mat_sel),
        .adr(adr),
        //.in_or_w1(in_or_w1),
        .w2(w2),
        .decision(decision)
        //.rout_data(rout_data),
        //.rd_en(rd_en),
        //.wr_en(wr_en)
        
        //.wstrb(wstrb),
        //.la_write(la_write),
        //.la_input(la_data_in[63:32]),
        //.count(count)
    );

endmodule


module mul_matr #(
    parameter BITS =32
) (
               input clk, 
               input [1:0] mat_sel, 
               input [1:0]adr,
               //input[10:0] in_or_w1,
               input signed [20:0] w2, 
               output reg [2:0] decision 
               );


reg signed [10:0] mem1 [2:0][2:0]; //input matrix
reg signed [10:0] mem2 [2:0][2:0]; //weight matrix/hiddenlayer
reg signed [20:0] mem3 [2:0][2:0]; //product matrix/hidden layer

reg signed [20:0] mem4 [2:0][2:0]; //output matrix of hidden layer
reg signed [20:0] mem5 [2:0][2:0]; //weight matrix/outputlayer
reg signed [40:0] mem6 [2:0][2:0]; // product matrix/output layer

integer i;
integer j;
integer k;

integer p;
integer q;

integer r;
integer s;

integer a,b,c;

integer e,f;
//integer u,v, ready;
reg [2:0] index;

reg signed [40:0] temp;
reg [2:0] temp1;

reg [2:0] new;

always@(posedge clk)
begin

//input matrix
//mem1[0][0]=-4;
//mem1[0][1]= 15;
//mem1[1][0]=2;
//mem1[1][1]= 4;

if(mat_sel==2'b00)
begin
	mem1[adr[1]][adr[0]]=w2;	
	
end
	
	
 if(mat_sel==2'b01)
		
begin
	
	mem2[adr[1]][adr[0]]=w2;	
	
end
	
	
	
if(mat_sel==2'b10)
		
begin
	mem5[adr[1]][adr[0]]=w2;	
	
end
	
	

// weight matrix hidden layer
//mem2[0][0]=20;
//mem2[0][1]= 15;
//mem2[1][0]=2;
//mem2[1][1]= 3;



// weight matrix output layer


//mem5[0][0]=100;
//mem5[0][1]= -25;
//mem5[1][0]=25;
//mem5[1][1]= 3;




//hidden layer calculations



//matrix initilization
for (p=0;p<=1;p=p+1)begin
  
  for(q=0;q<=1;q=q+1)begin
   
	mem3[p][q]=0;
	mem6[p][q]=0;

end

  end



//matrix multiplication

for (k=0;k<=1;k=k+1) begin

for (i=0; i<=1;i=i+1)begin
 
  for (j=0;j<=1;j=j+1) begin
    mem3[k][i]=mem3[k][i]+(mem1[k][j]*mem2[j][i]);

    end


end

end


//out1=mem3[x][y];


//relu

for(r=0;r<=1;r=r+1)begin

  for (s=0;s<=1;s=s+1) begin
   if(mem3[r][s]<=0)
	  mem4[r][s]=0;
	else if(mem3[r][s]>=0)
	   mem4[r][s]=mem3[r][s];
   

  end

end




//out2=mem4[x][y];



// output layer calculations




//matrix multiplication


for (a=0;a<=1;a=a+1) begin

for (b=0; b<=1;b=b+1)begin
 
  for (c=0;c<=1;c=c+1) begin
    mem6[a][b]=mem6[a][b]+(mem4[a][c]*mem5[c][b]);

    end


end

end



//out3=mem6[x][y];



// decision using comparison












temp=mem6[0][0];
index=0;
for (e=0;e<=1;e=e+1) begin
 
  for(f=0;f<=1;f=f+1) begin
    index=index+1;
   if (temp<=mem6[e][f]) begin
	
	  //temp=mem6[e][f];
	  temp1=index;
	end
  else if(temp>=mem6[e][f])  begin
       
       temp=temp;
  end
	
	
end


end

//decision=temp1;
//out4=temp;

new=temp;
decision=temp1;



end




endmodule



`default_nettype wire
