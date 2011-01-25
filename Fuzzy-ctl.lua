-------------------------------------------------------------------------------
--
-- Anubisway control
-- A self balancing two wheel robotic vehicle
--
-- March 2010, César Raúl Mamani Choquehuanca, PUC-Rio
-- Riobotz (www.riobotz.com.br)
-- Led Lab (www.giga.puc-rio.br)
--
-------------------------------------------------------------------------------
--Fuzzy Control
-------------------------------------------------------------------------------------
if pd.board()~="EK-LM3S8962" then
	print "Placa nao suportada"
	return
end
PI=math.pi
local adcvals={}				--matriz com os dados do ADC
FLAG=1							--0 REVERSE 1 FORWARD
f=io.open("/mmc/datasensor.txt","w") --Criando arquivo para gravar dados dos sensores

offsetzacc=577					--offset do acelerometro y
offsetgyr=872					--offset do gyroscopio (346.4) min=86.6 max=1646.35
offsetdir=498					--offset do potenciomentro

ang,vel=-15,0					--variaveis de inicio do filtro do angulo e velocidade
ang_ref=-15						--angulo de referenza em grau

KS=0.7							--ganho do potenciometro
direc=0							--Componente do potenciometro
--configuracao do adc
	ch={1,2,3}
	adcsmoothing={16,16,16}
	numiter=50
	for i,v in ipairs(ch) do
		adc.setblocking(v,1)
		adc.setclock(v,0)           --linha 10
		adc.setsmoothing(v,adcsmoothing[i])
	end
	--configuracao do pwm
	duty1,duty2=0,0
	pwmid1=4
	pwmid2=5
	pwm.setclock(pwmid1,5000)
	pwm.setclock(pwmid2,5000)
	clock1=5000
	clock2=5000
	pwm.setup(pwmid1,clock1,duty1)
	pwm.setup(pwmid2,clock2,duty2)
	pwm.start(pwmid1)
	pwm.start(pwmid2)
	--configurando o pin do inversor e o habilitador
	pio.pin.setdir( pio.OUTPUT, pio.PF_3 )	 --    SSICLK
	pio.pin.setdir( pio.OUTPUT, pio.PC_7 )	 --	 SFSS
	pio.pin.setdir( pio.OUTPUT, pio.PF_2 )	 --    SSIRX
	pio.pin.setdir( pio.OUTPUT, pio.PC_5 )	 --	 SSITX
	--Habilitacao dos motores
	pio.pin.setlow(pio.PF_2)				--SSITX low para abilitar
	pio.pin.setlow(pio.PC_5)				--SSIRX  low pra habilitazao
nume=11
numc=11
g1,g2,g0=20,12,100

we=0.2*g1
wc=0.2*g2
base=0.2*g0

ce={-1,-0.8,-0.6,-0.4,-0.2,0,0.2,0.4,0.6,0.8,1}
cc={-1,-0.6,-0.4,-0.2,-0.1,0,0.1,0.2,0.4,0.6,1}
rules={1 , 1   , 1   , 1   , 1   , 1   , 0.8 , 0.6 , 0.3 , 0.1 , 0;
       1 , 1   , 1   , 1   , 1   , 0.8 , 0.6 , 0.3 , 0.1 , 0   ,-0.1;
       1 , 1   , 1   , 1   , 0.8 , 0.6 , 0.3 , 0.1 , 0   ,-0.1 ,-0.2;
       1 , 1   , 1   , 0.8 , 0.6 , 0.3 , 0.1 , 0   ,-0.1 ,-0.2 ,-0.6;
       1 , 1   , 0.8 , 0.6 , 0.3 , 0.1 , 0   ,-0.1 ,-0.2 ,-0.6 ,-0.8;
	   1 , 0.8 , 0.6 , 0.3 , 0.1 , 0   ,-0.1 ,-0.2 ,-0.6 ,-0.8 , -1;
	 0.8 , 0.6 , 0.3 , 0.1 , 0   ,-0.1 ,-0.2 ,-0.6 ,-0.8 , -1  , -1;
	 0.6 , 0.3 , 0.1 , 0   ,-0.1 ,-0.2 ,-0.6 ,-0.8 , -1  , -1  , -1;
     0.3 , 0.1 , 0   ,-0.1 ,-0.2 ,-0.6 ,-0.8 , -1  , -1  , -1  , -1;
     0.1 , 0   , -0.1,-0.2 ,-0.6 ,-0.8 ,-1   , -1  , -1  , -1  , -1;
       0 , -0.1, -0.2,-0.6 ,-0.8 ,-1   ,-1   , -1  , -1  , -1  , -1}

mfe={0,0,0,0,0,0,0,0,0,0,0}
mfc={0,0,0,0,0,0,0,0,0,0,0}

for j=1,nume do
	ce[j]=ce[j]*g1
	cc[j]=cc[j]*g2
end
for j=1,numc*nume do
	rules[j]=(rules[j])*g0
end

e_count,c_count=0,0
e_int,c_int=0,0

while true do
	for j=1,numiter do
		adc.sample(ch,1)
		for i,v in ipairs(ch) do
			adc.insertsamples(v,adcvals,i,1)
		end
	end
	d_adc=adcvals[3] 							--direcao ADC3
	a_adc=adcvals[1] 							--acelerometro ADC1
	g_adc=adcvals[2] 							--giroscopio ADC2
	--Tratamento dos dados
	a_seg=math.floor((a_adc-offsetzacc)*0.75) 	--em graus seg
	g_seg=math.floor((g_adc-offsetgyr)*-0.35) 	--em graus seg/s
	d_seg=math.floor((d_adc-offsetdir)*0.1)		--em graus seg

	delta=tmr.read(0)
	ang=0.9*(ang+g_seg*delta*0.00000002) 		--filtragem do angulo
	delta=tmr.start(0)
	ang=ang+0.1*a_seg							--filtragem do angulo

	vel=0.85*vel+0.15*g_seg						--filtragem da velocidade
	direc=0.3*direc+0.7*d_seg					--filtragem da sinal de direcao
	direc=KS*direc

	e=ang_ref-ang								--erro de angulo
	edot=vel									--erro de velocidade
------------------------------------------------------------------------------------
-- 				Controle Fuzzy - Fuzzyficacao
------------------------------------------------------------------------------------
	if e<=ce[1] then
		mfe={1,0,0,0,0,0,0,0,0,0,0}
		e_count=e_count+1
		e_int=1
	elseif e>=ce[nume] then
		mfe={0,0,0,0,0,0,0,0,0,0,1}
		e_count=e_count+1
		e_int=nume
	else
		for i=1,nume do
			if e<=ce[i] then
				mfe[i]=math.max(0,1+(e-ce[i])/we)
				if mfe[i]~=0 then
					e_count=e_count+1
					e_int=i;
				end
			else
				mfe[i]=math.max(0,1+(ce[i]-e)/we)
				if mfe[i]~=0 then
					e_count=e_count+1
					e_int=i
				end
			end
		end
	end
	if edot<=cc[1] then
		mfc={1,0,0,0,0,0,0,0,0,0,0}
		c_count=c_count+1
		c_int=1
	elseif edot>=cc[numc] then
		mfc={0,0,0,0,0,0,0,0,0,0,1}
		c_count=c_count+1
		c_int=numc
	else
		for i=1,numc do
			if edot<=cc[i] then
				mfc[i]=math.max(0,1+(edot-cc[i])/wc)
				if mfc[i]~=0 then
					c_count=c_count+1
					c_int=i;
				end
			else
				mfc[i]=math.max(0,1+(cc[i]-edot)/wc)
				if mfc[i]~=0 then
					c_count=c_count+1
					c_int=i
				end
			end
		end
	end

	num=0
	den=0
------------------------------------------------------------------------------------
-- 				Controle Fuzzy - defuzzyficacao
------------------------------------------------------------------------------------
	for k=(e_int-e_count+1),e_int do
		for l=(c_int-c_count+1),c_int do
			prem=math.min(mfe[k],mfc[l])
			num=num+rules[(k-1)*nume+l]*base*(prem-(prem)^2/2)
			den=den+base*(prem-(prem)^2/2)
		end
	end
------------------------------------------------------------------------------------
-- 				Controle Fuzzy - End - U=num/dem eh a saida de controle
------------------------------------------------------------------------------------
	U=num/den
------------------------------------------------------------------------------------
	-- Salvando as leturas das variaveis de controle
	f:write(tostring(ang) .. " " .. tostring(vel) .. " " .. tostring(direc) .. " " .. tostring(e) .. "\n")
--enviando pra os motores
	duty1=U
	duty2=U

	if direc<6 and direc>-6 then
		direc=0
	end
	if direc>14 then
		direc=14
	elseif direc<-14 then
		direc=-14
	end
	duty1=duty1+direc
	duty2=duty2-direc
	duty1=math.floor(duty1)
	duty2=math.floor(duty2)

	if duty1<8 and duty1>-8 and FLAG==1 then
		duty1=8
	elseif duty1<8 and duty1>-8 and FLAG==0 then
		duty1=-8
	end
	if duty2<8 and duty2>-8 and FLAG==1 then
		duty2=8
	elseif duty2<8 and duty2>-8 and FLAG==0 then
		duty2=-8
	end
	if duty1>=8 then
		FLAG=1
		pio.pin.setlow(pio.PC_7)	--  motor ezquerdo se e positivo avanza low
	elseif duty1<=-8 then
		FLAG=0
		pio.pin.sethigh(pio.PC_7)	--  motor ezquerdo se e negativo retrocede high
	end
	if duty2>=8 then
		FLAG=1
		pio.pin.sethigh(pio.PF_3)	-- motor direito se e positivo avanza low
	elseif duty2<=-8 then
		FLAG=0
		pio.pin.setlow(pio.PF_3)	-- motor direito se e negativo retrocede high
	end
	duty1=math.abs(duty1)
	duty2=math.abs(duty2)
	if duty1>60 then
		duty1=60
	end
	if duty2>60 then
		duty2=60
	end
	pwm.setup(pwmid1,clock1,duty1)
	pwm.setup(pwmid2,clock2,duty2)
	e_count,c_count=0,0
	e_int,c_int=0,0

	if ang>22 then
		break
	elseif ang<-22 then
		break
	end

end
	--Fechando os motores
	pio.pin.sethigh(pio.PF_2)	-- high para deabilitar
	pio.pin.sethigh(pio.PC_5)
	f:close()
