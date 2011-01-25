-------------------------------------------------------------------------------
--
-- Anubisway control
-- A self balancing two wheel robotic vehicle
--
-- March 2010, CÃ©sar RaÃºl Mamani Choquehuanca, PUC-Rio
-- Riobotz (www.riobotz.com.br)
-- Led Lab (www.giga.puc-rio.br)
--
-------------------------------------------------------------------------------

--PID Control

--------------------------------------------------------------------------------


if pd.board()~="EK-LM3S8962" then
	print "Placa nao suportada"
	return
end
term.clrscr()
KP,KD,KS=5,4,0.3 		--constantes de controle PD e direcao
KI=0.70					--constante integral
I_acc=0					--Acumulador de integral
bufferint=0
local adcvals={} 		--matriz com os dados do ADC
ang,vel=5,0				--variaveis de inicio do filtro
U=0						--variavel de controle
FLAG=1					--0 REVERSE 1 FORWARD
direc=0

f=io.open("/mmc/datasensor.txt","w") --criando arquivo para salvar os dados dos sensores aquisitados

--variaveis offset para controle
offsetzacc=571		--offset do acelerometro y
offsetgyr=872		--offset do gyroscopio (346.4) min=86.6 max=1646.35
offsetdir=498		--offset do potenciomentro
ang_ref=5			--angulo de referenza em grau

--configuracao do adc
	ch={1,2,3}
	adcsmoothing={16,16,16}
	numiter=50
	for i,v in ipairs(ch) do
		adc.setblocking(v,1)
		adc.setclock(v,0)           --linha 10
		adc.setsmoothing(v,adcsmoothing[i])
	end
	numiter=50
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

--configuracao dos pinos de habilitação e reversa
	pio.pin.setdir( pio.OUTPUT, pio.PF_3 ) --    SSICLK
	pio.pin.setdir( pio.OUTPUT, pio.PC_7 ) --	 SFSS
	pio.pin.setdir( pio.OUTPUT, pio.PF_2 ) --    SSIRX
	pio.pin.setdir( pio.OUTPUT, pio.PC_5 ) --	 SSITX

--Habilitação dos motores
	pio.pin.setlow(pio.PF_2)	--SSITX low para abilitar
	pio.pin.setlow(pio.PC_5)	--SSIRX  low pra habilitazao

while true do      --inicio do loop
	--Aquisitando os dados do adc
	for j=1,numiter do
		adc.sample(ch,1)
		for i,v in ipairs(ch) do
			adc.insertsamples(v,adcvals,i,1)
		end
	end

	d_adc=adcvals[3] 	--direcao ADC3
	a_adc=adcvals[1] 	--acelerometro ADC1
	g_adc=adcvals[2] 	--giroscopio ADC2

	--Tratamento dos dados
	a_seg=math.floor((a_adc-offsetzacc)*0.75) 	--em graus seg
	g_seg=math.floor((g_adc-offsetgyr)*-0.35) 	--em graus seg/s
	d_seg=math.floor((d_adc-offsetdir)*0.1)		--em graus seg

	delta=tmr.read(0)
	ang=0.9*(ang+g_seg*delta*0.000002) 		--filtragem do angulo
	delta=tmr.start(0)
	ang=ang+0.1*a_seg						--filtragem do angulo

	vel=0.85*vel+0.15*g_seg					--filtragem da velocidade

	direc=0.6*direc+0.4*d_seg				--filtragem da sinal de direcao
	direc=KS*direc

------------------------------------------------------------------------------------
-- 				Controle PID - End - U  eh a saida de controle
------------------------------------------------------------------------------------
	I_acc=I_acc+(ang_ref-ang)
	if I_acc>15 then
		I_acc=15
	elseif I_acc<-15 then
		I_acc=-15
	end

	if bufferint*(ang_ref-ang)<0 then
		I_acc=0
	end

	bufferint=ang_ref-ang

	U=KP*(ang_ref-ang)+KD*vel+KI*I_acc
-----------------------------------------------------------------------------------

	-- Salvando as leturas das variaveis de controle
	f:write(tostring(ang) .. " " .. tostring(vel) .. " " .. tostring(direc) .. " " .. tostring(bufferint) .. "\n")

--enviando pra os motores
	duty1=U
	duty2=U

	if direc<8 and direc>-8 then
		direc=0
	end
	if direc>12 then
		direc=12
	elseif direc<-12 then
		direc=-12
	end
	duty1=duty1-direc
	duty2=duty2+direc
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
		pio.pin.sethigh(pio.PC_7)	--  motor ezquerdo se e positivo avanza low
	elseif duty1<=-8 then
		FLAG=0
		pio.pin.setlow(pio.PC_7)	--  motor ezquerdo se e negativo retrocede high
	end
	if duty2>=8 then
		FLAG=1
		pio.pin.setlow(pio.PF_3)	-- motor direito se e positivo avanza low
	elseif duty2<=-8 then
		FLAG=0
		pio.pin.sethigh(pio.PF_3)	-- motor direito se e negativo retrocede high
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

	if ang>18 then
		break
	elseif ang<-18 then
		break
	end
end
	--Fechando os motores
	pio.pin.sethigh(pio.PF_2)	--SSITX low para abilitar
	pio.pin.sethigh(pio.PC_5)	--SSIRX  low pra habilitazao
	f:close()

