/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef TEST_SCANS_CMP_H_
#define TEST_SCANS_CMP_H_

#ifdef __GNUC__
    #define MAYBE_UNUSED __attribute__((used))
#elif defined _MSC_VER
    #pragma warning(disable: Cxxxxx)
    #define MAYBE_UNUSED
#else
    #define MAYBE_UNUSED
#endif

double inf = GZ_DBL_INF;

static double __box_scan[] = {
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  1.5782255003,  1.5746054518,  1.5710810969,  1.5676514398,  1.5643155166,
  1.5610723943,  1.5579211705,  1.5548609726,  1.5518909572,  1.5490103095,
  1.5462182426,  1.5435139973,  1.5408968414,  1.5383660689,  1.5359210000,
  1.5335609804,  1.5312853811,  1.5290935973,  1.5269850490,  1.5249591799,
  1.5230154571,  1.5211533710,  1.5193724350,  1.5176721847,  1.5160521784,
  1.5145119959,  1.5130512390,  1.5116695307,  1.5103665155,  1.5091418586,
  1.5079952460,  1.5069263844,  1.5059350007,  1.5050208422,  1.5041836760,
  1.5034232892,  1.5027394889,  1.5021321015,  1.5016009732,  1.5011459694,
  1.5007669751,  1.5004638945,  1.5002366510,  1.5000851872,  1.5000094648,
  1.5000094648,  1.5000851872,  1.5002366510,  1.5004638945,  1.5007669751,
  1.5011459694,  1.5016009732,  1.5021321015,  1.5027394889,  1.5034232892,
  1.5041836760,  1.5050208422,  1.5059350007,  1.5069263844,  1.5079952460,
  1.5091418586,  1.5103665155,  1.5116695307,  1.5130512390,  1.5145119959,
  1.5160521784,  1.5176721847,  1.5193724350,  1.5211533710,  1.5230154571,
  1.5249591799,  1.5269850490,  1.5290935973,  1.5312853811,  1.5335609804,
  1.5359210000,  1.5383660689,  1.5408968414,  1.5435139973,  1.5462182426,
  1.5490103095,  1.5518909572,  1.5548609726,  1.5579211705,  1.5610723943,
  1.5643155166,  1.5676514398,  1.5710810969,  1.5746054518,  1.5782255003,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf};
static double *box_scan MAYBE_UNUSED = __box_scan;

static double __plane_scan[] = {
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,  9.6656585634,  9.0478807252,
  8.5047324589,  8.0234841835,  7.5941456600,  7.2087667472,  6.8609419394,
  6.5454526113,  6.2580042926,  5.9950307712,  5.7535460007,  5.5310307326,
  5.3253447326,  5.1346580881,  4.9573969333,  4.7922001805,  4.6378847381,
  4.4934173364,  4.3578915401,  4.2305088671,  4.1105631832,  3.9974277273,
  3.8905442644,  3.7894139705,  3.6935897344,  3.6026696258,  3.5162913303,
  3.4341273861,  3.3558810921,  3.2812829795,  3.2100877580,  3.1420716642,
  3.0770301506,  3.0147758675,  2.9551368928,  2.8979551762,  2.8430851689,
  2.7903926111,  2.7397534594,  2.6910529333,  2.6441846666,  2.5990499509,
  2.5555570588,  2.5136206380,  2.4731611670,  2.4341044661,  2.3963812565,
  2.3599267625,  2.3246803517,  2.2905852094,  2.2575880428,  2.2256388129,
  2.1946904899,  2.1646988309,  2.1356221764,  2.1074212653,  2.0800590642,
  2.0535006127,  2.0277128801,  2.0026646344,  1.9783263219,  1.9546699563,
  1.9316690161,  1.9092983502,  1.8875340914,  1.8663535750,  1.8457352644,
  1.8256586826,  1.8061043472,  1.7870537115,  1.7684891092,  1.7503937029,
  1.7327514359,  1.7155469886,  1.6987657360,  1.6823937094,  1.6664175601,
  1.6508245260,  1.6356023991,  1.6207394968,  1.6062246337,  1.5920470957,
  1.5781966157,  1.5646633508,  1.5514378607,  1.5385110875,  1.5258743370,
  1.5135192604,  1.5014378379,  1.4896223627,  1.4780654261,  1.4667599035,
  1.4556989409,  1.4448759425,  1.4342845592,  1.4239186765,  1.4137724050,
  1.4038400696,  1.3941162003,  1.3845955233,  1.3752729523,  1.3661435806,
  1.3572026733,  1.3484456602,  1.3398681287,  1.3314658175,  1.3232346102,
  1.3151705296,  1.3072697317,  1.2995285007,  1.2919432439,  1.2845104866,
  1.2772268677,  1.2700891354,  1.2630941425,  1.2562388432,  1.2495202886,
  1.2429356234,  1.2364820824,  1.2301569870,  1.2239577424,  1.2178818341,
  1.2119268257,  1.2060903552,  1.2003701333,  1.1947639402,  1.1892696236,
  1.1838850960,  1.1786083331,  1.1734373709,  1.1683703044,  1.1634052849,
  1.1585405190,  1.1537742661,  1.1491048371,  1.1445305925,  1.1400499409,
  1.1356613378,  1.1313632835,  1.1271543224,  1.1230330412,  1.1189980678,
  1.1150480699,  1.1111817542,  1.1073978648,  1.1036951824,  1.1000725232,
  1.0965287377,  1.0930627101,  1.0896733570,  1.0863596268,  1.0831204985,
  1.0799549813,  1.0768621133,  1.0738409614,  1.0708906196,  1.0680102095,
  1.0651988785,  1.0624557997,  1.0597801715,  1.0571712163,  1.0546281804,
  1.0521503335,  1.0497369679,  1.0473873979,  1.0451009599,  1.0428770110,
  1.0407149295,  1.0386141136,  1.0365739817,  1.0345939715,  1.0326735396,
  1.0308121617,  1.0290093316,  1.0272645609,  1.0255773792,  1.0239473333,
  1.0223739870,  1.0208569207,  1.0193957316,  1.0179900327,  1.0166394533,
  1.0153436380,  1.0141022473,  1.0129149566,  1.0117814565,  1.0107014523,
  1.0096746639,  1.0087008260,  1.0077796872,  1.0069110104,  1.0060945724,
  1.0053301640,  1.0046175896,  1.0039566671,  1.0033472281,  1.0027891173,
  1.0022821928,  1.0018263259,  1.0014214010,  1.0010673155,  1.0007639796,
  1.0005113168,  1.0003092630,  1.0001577673,  1.0000567915,  1.0000063099,
  1.0000063099,  1.0000567915,  1.0001577673,  1.0003092630,  1.0005113168,
  1.0007639796,  1.0010673155,  1.0014214010,  1.0018263259,  1.0022821928,
  1.0027891173,  1.0033472281,  1.0039566671,  1.0046175896,  1.0053301640,
  1.0060945724,  1.0069110104,  1.0077796872,  1.0087008260,  1.0096746639,
  1.0107014523,  1.0117814565,  1.0129149566,  1.0141022473,  1.0153436380,
  1.0166394533,  1.0179900327,  1.0193957316,  1.0208569207,  1.0223739870,
  1.0239473333,  1.0255773792,  1.0272645609,  1.0290093316,  1.0308121617,
  1.0326735396,  1.0345939715,  1.0365739817,  1.0386141136,  1.0407149295,
  1.0428770110,  1.0451009599,  1.0473873979,  1.0497369679,  1.0521503335,
  1.0546281804,  1.0571712163,  1.0597801715,  1.0624557997,  1.0651988785,
  1.0680102095,  1.0708906196,  1.0738409614,  1.0768621133,  1.0799549813,
  1.0831204985,  1.0863596268,  1.0896733570,  1.0930627101,  1.0965287377,
  1.1000725232,  1.1036951824,  1.1073978648,  1.1111817542,  1.1150480699,
  1.1189980678,  1.1230330412,  1.1271543224,  1.1313632835,  1.1356613378,
  1.1400499409,  1.1445305925,  1.1491048371,  1.1537742661,  1.1585405190,
  1.1634052849,  1.1683703044,  1.1734373709,  1.1786083331,  1.1838850960,
  1.1892696236,  1.1947639402,  1.2003701333,  1.2060903552,  1.2119268257,
  1.2178818341,  1.2239577424,  1.2301569870,  1.2364820824,  1.2429356234,
  1.2495202886,  1.2562388432,  1.2630941425,  1.2700891354,  1.2772268677,
  1.2845104866,  1.2919432439,  1.2995285007,  1.3072697317,  1.3151705296,
  1.3232346102,  1.3314658175,  1.3398681287,  1.3484456602,  1.3572026733,
  1.3661435806,  1.3752729523,  1.3845955233,  1.3941162003,  1.4038400696,
  1.4137724050,  1.4239186765,  1.4342845592,  1.4448759425,  1.4556989409,
  1.4667599035,  1.4780654261,  1.4896223627,  1.5014378379,  1.5135192604,
  1.5258743370,  1.5385110875,  1.5514378607,  1.5646633508,  1.5781966157,
  1.5920470957,  1.6062246337,  1.6207394968,  1.6356023991,  1.6508245260,
  1.6664175601,  1.6823937094,  1.6987657360,  1.7155469886,  1.7327514359,
  1.7503937029,  1.7684891092,  1.7870537115,  1.8061043472,  1.8256586826,
  1.8457352644,  1.8663535750,  1.8875340914,  1.9092983502,  1.9316690161,
  1.9546699563,  1.9783263219,  2.0026646344,  2.0277128801,  2.0535006127,
  2.0800590642,  2.1074212653,  2.1356221764,  2.1646988309,  2.1946904899,
  2.2256388129,  2.2575880428,  2.2905852094,  2.3246803517,  2.3599267625,
  2.3963812565,  2.4341044661,  2.4731611670,  2.5136206380,  2.5555570588,
  2.5990499509,  2.6441846666,  2.6910529333,  2.7397534594,  2.7903926111,
  2.8430851689,  2.8979551762,  2.9551368928,  3.0147758675,  3.0770301506,
  3.1420716642,  3.2100877580,  3.2812829795,  3.3558810921,  3.4341273861,
  3.5162913303,  3.6026696258,  3.6935897344,  3.7894139705,  3.8905442644,
  3.9974277273,  4.1105631832,  4.2305088671,  4.3578915401,  4.4934173364,
  4.6378847381,  4.7922001805,  4.9573969333,  5.1346580881,  5.3253447326,
  5.5310307326,  5.7535460007,  5.9950307712,  6.2580042926,  6.5454526113,
  6.8609419394,  7.2087667472,  7.5941456600,  8.0234841835,  8.5047324589,
  9.0478807252,  9.6656585634,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf,
  inf,           inf,           inf,           inf,           inf};
static double *plane_scan MAYBE_UNUSED = __plane_scan;

#endif
