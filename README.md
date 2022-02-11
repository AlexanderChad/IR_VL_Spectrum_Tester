# IR_VL_Spectrum_Tester

На основе Arduino nano и lcd дисплея 1620 был "на коленке" собран тестер спектральной проницаемости пленок.  
Данный прибор (скорее "показометр") после момещения между его приемником и свето-/ИК (940 нм) диодом (два диода, расположенные рядом)
показывает на экране: верхняя строка - коэффициент (отношение ИК к видимому спектру);
нижняя строка - проницаемость в процентах, слева - в ИК, а справа - в видимом спектре.  
Нажатие на кнопку запускает 3 замера с максимальной точностью коэффициента (слева сверху отображается номер замера). 
Данный режим активируется также через UART отправкой символа `a`.  
Калибровка производится отправкой через UART символа `c`. При калибровке приемник и передатчики должны находиться друг на против друга. Статус можно отслеживать в терминале.  
Может помочь в приблизительной оценке свойств светофильтров или пленок, например, при сравнении атермальных тонировочных пленок, а также само наличие атермального эффекта.

Прибор может работать и без дисплея. Через встроенный UART<->USB в Arduino nano на ПК (или даже смартфон) передаются данные вида:  
```
IR/VL Spect.Test is begin
IR_PWM:9152, IR:681.96, IR:096%, VL_PWM:24128, VL:704.49, VL:099%, Koef:0.97
...
IR_PWM:9152, IR:4.00, IR:000%, VL_PWM:24128, VL:41.15, VL:005%, Koef:0.10
...
UglAnalysis RUN!
IR_PWM:24856, IR:267.01, IR:037%, VL_PWM:65530, VL:4.00, VL:000%, Koef:66.75
...
```  
которые являются даже более исчерпывающими. `IR_PWM` - ШИМ для ИК, `VL_PWM` - для светодиода (ШИМ заполнение 0-65530); `IR` - проницаемость в ИК (0-1023 едениц АЦП и в процентах), 
`VL` - в видимом спектре (аналогично); `Koef` - коэффициент (отношение ИК к видимому спектру).

Схема:  
![Схема](https://github.com/AlexanderChad/IR_VL_Spectrum_Tester/blob/main/IR_VL_Spectrum_Tester_scheme.png)  

Тесты:  
![test1](https://github.com/AlexanderChad/IR_VL_Spectrum_Tester/blob/main/test1.jpg)  
![test2](https://github.com/AlexanderChad/IR_VL_Spectrum_Tester/blob/main/test2.jpg) 
![test3](https://github.com/AlexanderChad/IR_VL_Spectrum_Tester/blob/main/test3.jpg)  

P.S. пины выбраны так странно из-за того что у arduino, пошедшей под этот проект, неисправна большая часть пинов 
(при использовании в качестве логического анализатора в ремонте и, вынужденном реверс-инжиниринге, по невнимательности, на пины попало 24В 😥).
