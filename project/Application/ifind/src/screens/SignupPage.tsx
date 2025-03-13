import firestore from '@react-native-firebase/firestore';
import { NativeStackNavigationProp } from '@react-navigation/native-stack';
import React, { useState } from "react";
import { Alert, Image, StatusBar, StyleSheet, Text, TextInput, TouchableOpacity, View } from "react-native";
import { signUp } from '../lib/auth';
import theme from '../styles/theme';

interface SignupPageProps {
    navigation: NativeStackNavigationProp<any, 'SignupPage'>;
}

export const SignupPage = ({navigation}: SignupPageProps) => {
    const[form, setForm] = useState({
        email: "",
        password: "",
        confirmPassword: "",
        name: "",
        phone: "",
    })


    const handleInputChange = (name: string, value: string) => {
        setForm({
            ...form,
            [name]: value,
        });
    };

    const resultMessages = {
        "auth/email-already-in-use": "이미 가입된 이메일입니다.",
        "auth/wrong-password": "잘못된 비밀번호입니다.",
        "auth/user-not-found": "존재하지 않는 계정입니다.",
        "auth/invalid-email": "유효하지 않은 이메일 주소입니다."
    }


    interface FirebaseAuthError {
        code: keyof typeof resultMessages;
        message: string;
        }
    const signUpSubmit = async () => {
        const { email, password, confirmPassword, name, phone } = form;
        if (password !== confirmPassword) {
            Alert.alert("회원가입 실패", "비밀번호가 일치하지 않습니다.");
            return;
        }
        try {
            const userCredential = await signUp({ email, password });
            const user = userCredential.user;
            console.log(user);
            // 회원가입 성공하면
            await firestore().collection('users').doc(user.uid).set({
                name,
                phone,
            });
            navigation.navigate('LoginPage', {name: 'LoginPage'});
        } catch (e) {
            if (typeof e === "object" && e !== null && "code" in e) {
                const errorCode = (e as FirebaseAuthError).code;
                const alertMessage = resultMessages[errorCode] || "알 수 없는 이유로 회원가입에 실패하였습니다.";
                Alert.alert("회원가입 실패", alertMessage);
            } else {
                console.error("An unexpected error occurred:", e);
                Alert.alert("회원가입 실패", "알 수 없는 이유로 회원가입에 실패하였습니다.");
            }
        }
    };

    const goToLoginPage = () => {
        navigation.navigate('LoginPage', {name: 'LoginPage'});
    };

    return (
        <View style={styles.container}>
            <Image source={require("../asset/mainicon.png")} style={styles.logo_image}/>
            <Text style={styles.title}>어서오세요!</Text>
            <StatusBar hidden={true} />

            <View style={styles.inputContainer}>
                <TextInput style={styles.info} placeholder="이메일"
                placeholderTextColor='#3e3e3e'
                value={form.email}
                onChangeText={(value) => handleInputChange("email", value)}
                />
            </View>

            <View style={styles.inputContainer}>
                <TextInput style={styles.info} placeholder="비밀번호"
                placeholderTextColor='#3e3e3e'
                secureTextEntry={true}
                value={form.password}
                onChangeText={(value) => handleInputChange("password", value)}
                />
            </View>

            <View style={styles.inputContainer}>
                <TextInput style={styles.info} placeholder="비밀번호 확인"
                placeholderTextColor='#3e3e3e'
                secureTextEntry={true}
                value={form.confirmPassword}
                onChangeText={(value) => handleInputChange("confirmPassword", value)}
                />
            </View>

            <View style={styles.inputContainer}>
                <TextInput style={styles.info} placeholder="학생 이름"
                placeholderTextColor='#3e3e3e'
                value={form.name}
                onChangeText={(value) => handleInputChange("name", value)}
                />
            </View>

            <View style={styles.inputContainer}>
                <TextInput style={styles.info} placeholder="핸드폰번호" keyboardType="phone-pad"
                placeholderTextColor='#3e3e3e'
                value={form.phone}
                onChangeText={(value) => handleInputChange("phone", value)}
                />
            </View>

            {/* <View style={styles.inputContainer}>
                <TextInput style={styles.info} placeholder="주소"/>
            </View> */}

            <TouchableOpacity
                onPress={signUpSubmit}
                style={styles.submitButton}
            >
                <Text style={styles.submit}>회원가입</Text>
            </TouchableOpacity>
            
            <TouchableOpacity
                onPress={goToLoginPage}
            >
                <Text style={styles.gotologintext}>이미 회원이시면? 로그인</Text>
            </TouchableOpacity>
            
        </View>
    );
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
    },
    logo_image: {
        width: 80,
        height: 80,
    },
    gotologintext: {
        marginTop: 10,
        color:'gray',
        fontSize: 20,
        textDecorationLine: 'underline',
    },
    title: {
        fontSize: 23,
        fontFamily: theme.mainfont,
        // fontWeight: 'bold',
        marginBottom: 40,
        marginTop: 10,
        color: theme.mainDarkGrey,
    },
    inputContainer: {
        width: '85%',
        borderBottomWidth: 0.8,
        borderColor: theme.signupandin,
        color: theme.mainDarkGrey,
        marginBottom: 15,
    },
    info : {
        fontFamily: theme.mainfont,
        fontSize: 23,
        color: theme.mainDarkGrey,
    },
    submitButton: {
    width: '40%',
    alignItems: 'center',
    marginTop: 20,
    backgroundColor: theme.mainOrange,
    paddingHorizontal: 20,
    paddingVertical: 10,
    borderRadius: 40,
    shadowColor: "#000",
        shadowOffset: {
            width: 0,
            height: 2,
        },
        shadowOpacity: 0.25,
        shadowRadius: 4,
        elevation: 5,
    },
    submit: {
    fontFamily: theme.mainfont,
    padding: 5,
    color: 'black',
    fontSize: 23,
    },
});

