import 'styled-components';

declare module 'styled-components' {
    export interface DefaultTheme {
    mainBlue: string;
    mainDarkGrey: string;
    mainLightGrey: string;
    mainRed: string;
    mainOrange: string;
    mainGreen: string;

    fontLarge: number;
    fontMedium: number;
    fontRegular: number;
    fontSmall: number;

    weightBold: number;
    weightMedium: number;
    weightRegular: number;

    lineHeightRegular: number;
    lineHeightMicro: number;
    }
}