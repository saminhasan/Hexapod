    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 2;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 19;
            section.data(19)  = dumData; %prealloc

                    ;% rtP.time
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.uDLookupTable_tableData
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 10001;

                    ;% rtP.Gain_Gain
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 20002;

                    ;% rtP.uDLookupTable_tableData_ngisugqubh
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 20003;

                    ;% rtP.Gain1_Gain
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 30004;

                    ;% rtP.uDLookupTable_tableData_ai1chejlcf
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 30005;

                    ;% rtP.Gain_Gain_mphseqoivg
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 40006;

                    ;% rtP.uDLookupTable_tableData_l2sller51b
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 40007;

                    ;% rtP.Gain_Gain_cmu3wv0ql2
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 50008;

                    ;% rtP.uDLookupTable_tableData_osrraa44zy
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 50009;

                    ;% rtP.Gain_Gain_fhgg2tbdth
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 60010;

                    ;% rtP.uDLookupTable_tableData_nuebhjad2j
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 60011;

                    ;% rtP.Gain_Gain_g31zf1epte
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 70012;

                    ;% rtP.Constant_Value
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 70013;

                    ;% rtP.Constant_Value_bjkzcm1v50
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 70014;

                    ;% rtP.Constant_Value_cuijmfc0gq
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 70015;

                    ;% rtP.Constant_Value_h3mrzjkbom
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 70016;

                    ;% rtP.Constant_Value_heuixq11n0
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 70017;

                    ;% rtP.Constant_Value_jvff2kcwkv
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 70018;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 18;
            section.data(18)  = dumData; %prealloc

                    ;% rtP.uDLookupTable_maxIndex
                    section.data(1).logicalSrcIdx = 19;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.uDLookupTable_dimSizes
                    section.data(2).logicalSrcIdx = 20;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.uDLookupTable_numYWorkElts
                    section.data(3).logicalSrcIdx = 21;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.uDLookupTable_maxIndex_gecwmd542g
                    section.data(4).logicalSrcIdx = 22;
                    section.data(4).dtTransOffset = 4;

                    ;% rtP.uDLookupTable_dimSizes_dplxybs3xu
                    section.data(5).logicalSrcIdx = 23;
                    section.data(5).dtTransOffset = 5;

                    ;% rtP.uDLookupTable_numYWorkElts_pvvzvokkjh
                    section.data(6).logicalSrcIdx = 24;
                    section.data(6).dtTransOffset = 6;

                    ;% rtP.uDLookupTable_maxIndex_kmnxkox2xs
                    section.data(7).logicalSrcIdx = 25;
                    section.data(7).dtTransOffset = 8;

                    ;% rtP.uDLookupTable_dimSizes_gpxa1msfef
                    section.data(8).logicalSrcIdx = 26;
                    section.data(8).dtTransOffset = 9;

                    ;% rtP.uDLookupTable_numYWorkElts_bgs0ock0jg
                    section.data(9).logicalSrcIdx = 27;
                    section.data(9).dtTransOffset = 10;

                    ;% rtP.uDLookupTable_maxIndex_ewjlqfurjs
                    section.data(10).logicalSrcIdx = 28;
                    section.data(10).dtTransOffset = 12;

                    ;% rtP.uDLookupTable_dimSizes_jln4ytncld
                    section.data(11).logicalSrcIdx = 29;
                    section.data(11).dtTransOffset = 13;

                    ;% rtP.uDLookupTable_numYWorkElts_pjunzd1vya
                    section.data(12).logicalSrcIdx = 30;
                    section.data(12).dtTransOffset = 14;

                    ;% rtP.uDLookupTable_maxIndex_imwny5u4rw
                    section.data(13).logicalSrcIdx = 31;
                    section.data(13).dtTransOffset = 16;

                    ;% rtP.uDLookupTable_dimSizes_fvopwqzbwe
                    section.data(14).logicalSrcIdx = 32;
                    section.data(14).dtTransOffset = 17;

                    ;% rtP.uDLookupTable_numYWorkElts_iiusdsomba
                    section.data(15).logicalSrcIdx = 33;
                    section.data(15).dtTransOffset = 18;

                    ;% rtP.uDLookupTable_maxIndex_o1qqvqqib2
                    section.data(16).logicalSrcIdx = 34;
                    section.data(16).dtTransOffset = 20;

                    ;% rtP.uDLookupTable_dimSizes_nyynambagj
                    section.data(17).logicalSrcIdx = 35;
                    section.data(17).dtTransOffset = 21;

                    ;% rtP.uDLookupTable_numYWorkElts_on1gheggyj
                    section.data(18).logicalSrcIdx = 36;
                    section.data(18).dtTransOffset = 22;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 14;
            section.data(14)  = dumData; %prealloc

                    ;% rtB.aslpipoci2
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.fzxmotide1
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtB.kfjnjnbiln
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtB.bkamu4msu4
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtB.ecpp33hi3p
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtB.gtyqmeteln
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtB.gmkqayjiuv
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtB.du1olrgvrn
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 10;

                    ;% rtB.mcnp4idm3j
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 14;

                    ;% rtB.fns5dbtm53
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 18;

                    ;% rtB.fq1dlz2two
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 22;

                    ;% rtB.lssf54mn5z
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 26;

                    ;% rtB.lgfnqwstci
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 30;

                    ;% rtB.i4kzipy3mq
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 79;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 7;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 54;
            section.data(54)  = dumData; %prealloc

                    ;% rtDW.o35huj2gsd
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.klzhkf4szf
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.dagi3ssnao
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.eeo3dalye1
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.idl3np2sad
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.a0lmv0bgi0
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.fmzddgcets
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.blbm45o2uq
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.mbr2ermefj
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.dnv5nxxb53
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.hklg4jlt4z
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtDW.mjt4xoczh4
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtDW.ev2m2wgy2j
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% rtDW.fg5gnsupey
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% rtDW.nopt2maj0d
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% rtDW.nukyf0pyaa
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% rtDW.comai21xuo
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% rtDW.batgpdl5hb
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 10017;

                    ;% rtDW.darmnwyzix
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 20018;

                    ;% rtDW.plfvcgjov5
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 20019;

                    ;% rtDW.i4pwvk5fud
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20020;

                    ;% rtDW.empqd1cr2q
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 20021;

                    ;% rtDW.jge2c5avbd
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 20022;

                    ;% rtDW.k1dqwarsjy
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 30023;

                    ;% rtDW.cp3e04jgmd
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 40024;

                    ;% rtDW.i4rnfp1wv3
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 40025;

                    ;% rtDW.dpbxldkwzj
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 40026;

                    ;% rtDW.e2ooci3cok
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 40027;

                    ;% rtDW.bciralxndx
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 40028;

                    ;% rtDW.j0h5r2rwwx
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 50029;

                    ;% rtDW.dcbjunmnuz
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 60030;

                    ;% rtDW.d0piouqmcg
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 60031;

                    ;% rtDW.b3exesx2ov
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 60032;

                    ;% rtDW.k4irirzhzl
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 60033;

                    ;% rtDW.nadia4sijt
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 60034;

                    ;% rtDW.btlye4fecs
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 70035;

                    ;% rtDW.ap0qucs3ix
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 80036;

                    ;% rtDW.clnubtij1c
                    section.data(38).logicalSrcIdx = 37;
                    section.data(38).dtTransOffset = 80037;

                    ;% rtDW.ga4gvlvb3k
                    section.data(39).logicalSrcIdx = 38;
                    section.data(39).dtTransOffset = 80038;

                    ;% rtDW.fdnrequow3
                    section.data(40).logicalSrcIdx = 39;
                    section.data(40).dtTransOffset = 80039;

                    ;% rtDW.esd12dsf3p
                    section.data(41).logicalSrcIdx = 40;
                    section.data(41).dtTransOffset = 80040;

                    ;% rtDW.lw1aa3ikun
                    section.data(42).logicalSrcIdx = 41;
                    section.data(42).dtTransOffset = 90041;

                    ;% rtDW.hig0ipabbz
                    section.data(43).logicalSrcIdx = 42;
                    section.data(43).dtTransOffset = 100042;

                    ;% rtDW.d2a3xixuf0
                    section.data(44).logicalSrcIdx = 43;
                    section.data(44).dtTransOffset = 100043;

                    ;% rtDW.nsivhr4agq
                    section.data(45).logicalSrcIdx = 44;
                    section.data(45).dtTransOffset = 100044;

                    ;% rtDW.hiwcepl5bw
                    section.data(46).logicalSrcIdx = 45;
                    section.data(46).dtTransOffset = 100045;

                    ;% rtDW.dm5bb5xb02
                    section.data(47).logicalSrcIdx = 46;
                    section.data(47).dtTransOffset = 100046;

                    ;% rtDW.a4gdm1csbd
                    section.data(48).logicalSrcIdx = 47;
                    section.data(48).dtTransOffset = 110047;

                    ;% rtDW.hv225z33ok
                    section.data(49).logicalSrcIdx = 48;
                    section.data(49).dtTransOffset = 120048;

                    ;% rtDW.c0mmrfri1b
                    section.data(50).logicalSrcIdx = 49;
                    section.data(50).dtTransOffset = 120049;

                    ;% rtDW.ejjqivakel
                    section.data(51).logicalSrcIdx = 50;
                    section.data(51).dtTransOffset = 120050;

                    ;% rtDW.duhp5c0dw3
                    section.data(52).logicalSrcIdx = 51;
                    section.data(52).dtTransOffset = 120051;

                    ;% rtDW.mzkea3zvvh
                    section.data(53).logicalSrcIdx = 52;
                    section.data(53).dtTransOffset = 120052;

                    ;% rtDW.orrqp05dyy
                    section.data(54).logicalSrcIdx = 53;
                    section.data(54).dtTransOffset = 120053;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 39;
            section.data(39)  = dumData; %prealloc

                    ;% rtDW.jfocsgvjr2
                    section.data(1).logicalSrcIdx = 54;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.etlkslxoll
                    section.data(2).logicalSrcIdx = 55;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.lcknyo52qu
                    section.data(3).logicalSrcIdx = 56;
                    section.data(3).dtTransOffset = 7;

                    ;% rtDW.bwgtinpdi5
                    section.data(4).logicalSrcIdx = 57;
                    section.data(4).dtTransOffset = 16;

                    ;% rtDW.glqjd0j2v1
                    section.data(5).logicalSrcIdx = 58;
                    section.data(5).dtTransOffset = 17;

                    ;% rtDW.dqohor0ufz
                    section.data(6).logicalSrcIdx = 59;
                    section.data(6).dtTransOffset = 23;

                    ;% rtDW.iha3lbqkcg
                    section.data(7).logicalSrcIdx = 60;
                    section.data(7).dtTransOffset = 32;

                    ;% rtDW.fee4e2lshm
                    section.data(8).logicalSrcIdx = 61;
                    section.data(8).dtTransOffset = 33;

                    ;% rtDW.k4afwk3pmb
                    section.data(9).logicalSrcIdx = 62;
                    section.data(9).dtTransOffset = 39;

                    ;% rtDW.cayhrhdjj2
                    section.data(10).logicalSrcIdx = 63;
                    section.data(10).dtTransOffset = 48;

                    ;% rtDW.eh4jhgdpjq
                    section.data(11).logicalSrcIdx = 64;
                    section.data(11).dtTransOffset = 49;

                    ;% rtDW.nlcxo5f5f4
                    section.data(12).logicalSrcIdx = 65;
                    section.data(12).dtTransOffset = 55;

                    ;% rtDW.fza5104ubd
                    section.data(13).logicalSrcIdx = 66;
                    section.data(13).dtTransOffset = 64;

                    ;% rtDW.bot0f4jvu3
                    section.data(14).logicalSrcIdx = 67;
                    section.data(14).dtTransOffset = 65;

                    ;% rtDW.nvuzcubtym
                    section.data(15).logicalSrcIdx = 68;
                    section.data(15).dtTransOffset = 71;

                    ;% rtDW.og4xbrvaxl
                    section.data(16).logicalSrcIdx = 69;
                    section.data(16).dtTransOffset = 80;

                    ;% rtDW.gsuie3xmsc
                    section.data(17).logicalSrcIdx = 70;
                    section.data(17).dtTransOffset = 81;

                    ;% rtDW.kgv4ez4phl
                    section.data(18).logicalSrcIdx = 71;
                    section.data(18).dtTransOffset = 87;

                    ;% rtDW.ngbzoo1n2w
                    section.data(19).logicalSrcIdx = 72;
                    section.data(19).dtTransOffset = 96;

                    ;% rtDW.cn01mlmqpq
                    section.data(20).logicalSrcIdx = 73;
                    section.data(20).dtTransOffset = 97;

                    ;% rtDW.dzz3fjwjh4
                    section.data(21).logicalSrcIdx = 74;
                    section.data(21).dtTransOffset = 98;

                    ;% rtDW.pmulv1s4uc
                    section.data(22).logicalSrcIdx = 75;
                    section.data(22).dtTransOffset = 99;

                    ;% rtDW.dqtv4j00nz
                    section.data(23).logicalSrcIdx = 76;
                    section.data(23).dtTransOffset = 100;

                    ;% rtDW.lyxq4lljok
                    section.data(24).logicalSrcIdx = 77;
                    section.data(24).dtTransOffset = 101;

                    ;% rtDW.a3motogwve
                    section.data(25).logicalSrcIdx = 78;
                    section.data(25).dtTransOffset = 102;

                    ;% rtDW.o0r4inb335
                    section.data(26).logicalSrcIdx = 79;
                    section.data(26).dtTransOffset = 103;

                    ;% rtDW.edpwaq1mh5
                    section.data(27).logicalSrcIdx = 80;
                    section.data(27).dtTransOffset = 104;

                    ;% rtDW.i523mfvkwd
                    section.data(28).logicalSrcIdx = 81;
                    section.data(28).dtTransOffset = 105;

                    ;% rtDW.a0jpeenedk.AQHandles
                    section.data(29).logicalSrcIdx = 82;
                    section.data(29).dtTransOffset = 106;

                    ;% rtDW.hpsgqxjk3e.AQHandles
                    section.data(30).logicalSrcIdx = 83;
                    section.data(30).dtTransOffset = 107;

                    ;% rtDW.iaizpaoviz.AQHandles
                    section.data(31).logicalSrcIdx = 84;
                    section.data(31).dtTransOffset = 108;

                    ;% rtDW.agizt5nhn4.AQHandles
                    section.data(32).logicalSrcIdx = 85;
                    section.data(32).dtTransOffset = 109;

                    ;% rtDW.f54c0stnai.AQHandles
                    section.data(33).logicalSrcIdx = 86;
                    section.data(33).dtTransOffset = 110;

                    ;% rtDW.fhypfegxnq.AQHandles
                    section.data(34).logicalSrcIdx = 87;
                    section.data(34).dtTransOffset = 111;

                    ;% rtDW.m1elosxkow
                    section.data(35).logicalSrcIdx = 88;
                    section.data(35).dtTransOffset = 112;

                    ;% rtDW.lhzhdx15na
                    section.data(36).logicalSrcIdx = 89;
                    section.data(36).dtTransOffset = 113;

                    ;% rtDW.ljxcsqwcit
                    section.data(37).logicalSrcIdx = 90;
                    section.data(37).dtTransOffset = 114;

                    ;% rtDW.h4jnxgasty
                    section.data(38).logicalSrcIdx = 91;
                    section.data(38).dtTransOffset = 115;

                    ;% rtDW.asduixo1nh
                    section.data(39).logicalSrcIdx = 92;
                    section.data(39).dtTransOffset = 116;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% rtDW.fusdopd4md
                    section.data(1).logicalSrcIdx = 93;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.lbojcaqzep
                    section.data(2).logicalSrcIdx = 94;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.ozm0zgbvmf
                    section.data(3).logicalSrcIdx = 95;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.jdsiaq5mqq
                    section.data(4).logicalSrcIdx = 96;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.gyvass5xmg
                    section.data(5).logicalSrcIdx = 97;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.hcjb1vjnr1
                    section.data(6).logicalSrcIdx = 98;
                    section.data(6).dtTransOffset = 5;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% rtDW.bmm15pnlh2
                    section.data(1).logicalSrcIdx = 99;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.lu02c5leuy
                    section.data(2).logicalSrcIdx = 100;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.chb23a0xlk
                    section.data(3).logicalSrcIdx = 101;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.cheuqjrosp
                    section.data(4).logicalSrcIdx = 102;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.bbm34zsyfk
                    section.data(5).logicalSrcIdx = 103;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.juxxtpbgcs
                    section.data(6).logicalSrcIdx = 104;
                    section.data(6).dtTransOffset = 5;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% rtDW.ee11ecv44v
                    section.data(1).logicalSrcIdx = 105;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.cu4ltknb50
                    section.data(2).logicalSrcIdx = 106;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.arrnm4nhoz
                    section.data(3).logicalSrcIdx = 107;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.mskbr31frt
                    section.data(4).logicalSrcIdx = 108;
                    section.data(4).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 10;
            section.data(10)  = dumData; %prealloc

                    ;% rtDW.irnshlasoh
                    section.data(1).logicalSrcIdx = 109;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.k3w4ik3bnp
                    section.data(2).logicalSrcIdx = 110;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.az4qyz5ina
                    section.data(3).logicalSrcIdx = 111;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.d5ypu3fzep
                    section.data(4).logicalSrcIdx = 112;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.asbkawbx5i
                    section.data(5).logicalSrcIdx = 113;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.pfmvz0ju2f
                    section.data(6).logicalSrcIdx = 114;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.jjr0g32epl
                    section.data(7).logicalSrcIdx = 115;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.m1v51w4plc
                    section.data(8).logicalSrcIdx = 116;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.bwaatfgl13
                    section.data(9).logicalSrcIdx = 117;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.ghbw0wjs25
                    section.data(10).logicalSrcIdx = 118;
                    section.data(10).dtTransOffset = 9;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.bdszturt0n
                    section.data(1).logicalSrcIdx = 119;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.mngswn4g4e
                    section.data(2).logicalSrcIdx = 120;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 2301247433;
    targMap.checksum1 = 1297405112;
    targMap.checksum2 = 3492532633;
    targMap.checksum3 = 2774462004;

