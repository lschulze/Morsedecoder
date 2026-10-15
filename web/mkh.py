import gzip
d=gzip.compress(open('page.html','rb').read(),9,mtime=0)
s='// v51: Design C, Laufband langsam + grosse Schrift, gzip (generiert von web/mkh.py aus web/page.html)\n#pragma once\nconst size_t WEB_PAGE_GZ_LEN = %d;\nconst uint8_t WEB_PAGE_GZ[] PROGMEM = {\n'%len(d)
s+=',\n'.join(','.join('0x%02x'%b for b in d[i:i+24]) for i in range(0,len(d),24))+'\n};\n'
open('../morse_esp32/web_page.h','w').write(s); print(len(d))
